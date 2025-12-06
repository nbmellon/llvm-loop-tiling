//===-- Frequent Path Loop Invariant Code Motion Pass --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===---------------------------------------------------------------------===//
//
// CSE583 F25 - This pass can be used as a template for your FPLICM homework
//               assignment.
//               The passes get registered as "fplicm-correctness" and
//               "fplicm-performance".
//
//
////===-------------------------------------------------------------------===//
#include "llvm/Analysis/BlockFrequencyInfo.h"
#include "llvm/Analysis/BranchProbabilityInfo.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/LoopIterator.h"
#include "llvm/Analysis/LoopPass.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/PassManager.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Passes/PassPlugin.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/Scalar/LoopPassManager.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Transforms/Utils/LoopUtils.h"

// Additional headers
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/Operator.h"
#include "llvm/Support/Casting.h"
#include <vector>
#include <algorithm>
#include <cmath>

using namespace llvm;

/// Helper: try to extract a vector of SCEV expressions for
/// the subscripts used by a pointer Value (for multi-dim arrays).
/// This is conservative; if we can't decompose, returns empty vector.
/// We assume canonical form: getElementPtr or simple arithmetic.
static bool extractSubscripSCEVs(Value *Ptr,
                                 ScalarEvolution &SE,
                                 SmallVectorImpl<const SCEV*> &Out) {
  // Handle GetElementPtrInst / ConstantExpr GEP
  if (GetElementPtrInst *GEP = dyn_cast<GetElementPtrInst>(Ptr)) {
    for (auto &idx : GEP->indices()) {
      const SCEV *s = SE.getSCEV(idx.get());
      Out.push_back(s);
    }
    return !Out.empty();
  }
  // If pointer is result of add/mul, we can attempt to look at the operand
  // as fallback; conservative: return failure so callers treat as dependent.
  return false;
}

/// Determine if an SCEV is loop-invariant w.r.t. Loop L.
static bool isInvariantWRTLoop(const SCEV *S, Loop *L) {
  // SCEV::isLoopInvariant is the right way (we need to check)
  // but we must guard S != nullptr
  if (!S) return false;
  return S->isLoopInvariant(L);
}

/// Heuristic: determine if an SCEV is "stride-1 w.r.t loop L only" i.e.,
/// it depends on loop L with coefficient ±1 and does not depend on other
/// loops in the band. This is a heuristic used to detect stride-1 accesses.
static bool isStrideOneWRTLoop(const SCEV *S, Loop *L, ScalarEvolution &SE) {
  // If S is an AddRecExpr, we can inspect the step.
  if (const SCEVAddRecExpr *AR = dyn_cast<SCEVAddRecExpr>(S)) {
    // If this AddRec carries loop 'L' then AR->getLoop() == L
    if (AR->getLoop() != L) return false;
    // step is AR->getStepRecurrence(SE)
    const SCEV *Step = AR->getStepRecurrence(SE);
    // Simple check: step is SCEVConstant(1) or -1
    if (const SCEVConstant *C = dyn_cast<SCEVConstant>(Step)) {
      const APInt &val = C->getAPInt();
      // check if absolute value == 1
      return (val == 1) || (val == -1);
    }
    return false;
  }

  // If it's not an AddRec (parametric) but an affine expression a*iv + b,
  // we attempt to see if the SCEV can be expressed as Mul(Constant, AddRec)
  // Conservatively return false.
  return false;
}

/// Compute dimensional reuse γ for a permutable band G (vector of Loop*).
/// Returns vector<double> of size = G.size() with γ per loop in G.
///
/// Analysis objects required: ScalarEvolution &SE, LoopInfo &LI
static std::vector<double> computeDimensionalReuse(ArrayRef<Loop*> G,
                                                   ScalarEvolution &SE) {
  unsigned N = G.size();
  std::vector<double> rawReuse(N, 0.0);

  // Walk all instructions in the loops in G:
  // conservative: collect all basic blocks in the innermost loop(s)
  // We will iterate over all loops in band and their blocks to find loads/stores.
  SmallPtrSet<BasicBlock*, 16> visitedBlocks;
  SmallVector<Instruction*, 64> memInsts;

  for (Loop *L : G) {
    for (BasicBlock *BB : L->getBlocks()) {
      if (!visitedBlocks.insert(BB).second) continue;
      for (Instruction &I : *BB) {
        if (isa<LoadInst>(&I) || isa<StoreInst>(&I)) {
          memInsts.push_back(&I);
        }
      }
    }
  }

  // For each memory instruction, get pointer's SCEV subscripts and reason
  for (Instruction *I : memInsts) {
    Value *ptr = nullptr;
    bool isStore = false;
    if (LoadInst *LI = dyn_cast<LoadInst>(I)) {
      ptr = LI->getPointerOperand();
      isStore = false;
    } else if (StoreInst *SI = dyn_cast<StoreInst>(I)) {
      ptr = SI->getPointerOperand();
      isStore = true;
    } else {
      continue;
    }

    SmallVector<const SCEV*, 8> subs;
    bool success = extractSubscripSCEVs(ptr, SE, subs);

    // Conservative: if we couldn't extract subscripts (complex ptr),
    // treat the access as dependent on all loops (no temporal reuse).
    if (!success) {
      // But we can still try to examine if pointer is loop-invariant
      const SCEV *s = SE.getSCEV(ptr);
      for (unsigned d = 0; d < N; ++d) {
        Loop *Ld = G[d];
        if (isInvariantWRTLoop(s, Ld)) {
          // Pointer invariant => temporal reuse for this loop
          rawReuse[d] += 1.0;
          if (isStore) rawReuse[d] += 1.0; // extra weight for write
        } else {
          // dependent => maybe spatial reuse, but if unknown, give small weight
          rawReuse[d] += 0.0; // conservative: don't add
        }
      }
      continue;
    }

    // If we have subscripts, analyze each subscript expression for dependence
    // We'll mark per-dimension contributions:
    SmallVector<double, 8> localContribution(N, 0.0);

    for (const SCEV *sub : subs) {
      // If sub is loop invariant w.r.t some Ld, that means the array index
      // doesn't vary with that loop and hence the array exhibits temporal reuse
      for (unsigned d = 0; d < N; ++d) {
        Loop *Ld = G[d];
        if (isInvariantWRTLoop(sub, Ld)) {
          localContribution[d] += 1.0; // temporal reuse contribution
        } else {
          // If sub is a AddRec for the loop and step == 1, treat as stride-1 spatial reuse
          if (isStrideOneWRTLoop(sub, Ld, SE)) {
            localContribution[d] += 1.0; // spatial reuse contributes towards dim reuse
          } else {
            // dependent but not stride-1 => less helpful for reuse; small contribution 0
            localContribution[d] += 0.0;
          }
        }
      }
    } // end each subscript

    // If this memref is a store + load (reductions/updates), give extra weight to the
    // loops where it was invariant (paper increases k to 2 in matmul example).
    if (isStore) {
      for (unsigned d = 0; d < N; ++d) localContribution[d] += 1.0; // weight writes
    }

    // Add local contributions to rawReuse
    for (unsigned d = 0; d < N; ++d) rawReuse[d] += localContribution[d];
  } // end each mem instruction

  // Normalization: divide by max so that the largest reuse dimension becomes 1.0.
  double maxVal = 0.0;
  for (double v : rawReuse) if (v > maxVal) maxVal = v;
  std::vector<double> gamma(N, 0.0);
  if (maxVal <= 0.0) {
    // No reuse detected. As fallback, assign equal small reuse to avoid zeros.
    for (unsigned i = 0; i < N; ++i) gamma[i] = 1.0 / double(N);
    return gamma;
  }
  for (unsigned i = 0; i < N; ++i) gamma[i] = rawReuse[i] / maxVal;
  return gamma;
}


namespace {
  struct HW2CorrectnessPass : public PassInfoMixin<HW2CorrectnessPass> {

    PreservedAnalyses run(Function &F, FunctionAnalysisManager &FAM) {
      llvm::BlockFrequencyAnalysis::Result &bfi = FAM.getResult<BlockFrequencyAnalysis>(F);
      llvm::BranchProbabilityAnalysis::Result &bpi = FAM.getResult<BranchProbabilityAnalysis>(F);
      llvm::LoopAnalysis::Result &li = FAM.getResult<LoopAnalysis>(F);
      /* *******Implementation Starts Here******* */
      // Your core logic should reside here.

      ScalarEvolution &SE = FAM.getResult<ScalarEvolutionAnalysis>(F);
      // std::vector<Loop*> bandLoops = /* from step 1 */;
      std::vector<double> gamma = computeDimensionalReuse(bandLoops, SE);


      /* *******Implementation Ends Here******* */
      // Your pass is modifying the source code. Figure out which analyses
      // are preserved and only return those, not all.
      return PreservedAnalyses::all();
    }
  };
}

extern "C" ::llvm::PassPluginLibraryInfo LLVM_ATTRIBUTE_WEAK llvmGetPassPluginInfo() {
  return {
    LLVM_PLUGIN_API_VERSION, "HW2Pass", "v0.1",
    [](PassBuilder &PB) {
      PB.registerPipelineParsingCallback(
        [](StringRef Name, FunctionPassManager &FPM,
        ArrayRef<PassBuilder::PipelineElement>) {
          if(Name == "fplicm-correctness"){
            FPM.addPass(HW2CorrectnessPass());
            return true;
          }
          return false;
        }
      );
    }
  };
}
