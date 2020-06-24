#ifndef SOLVER_ADMM_H
#define SOLVER_ADMM_H

#include <glog/logging.h>

#include <iomanip>
#include <iostream>

#include "solver/solver_option.h"
#include "solver/summary.h"

namespace DAGSfM {

// An abstract class for Alternating Direction Method of Multipliers (ADMM)
// Any class which want to implement a solver that based on ADMM should
// inherit this class

class ADMM {
  // public:
  // virtual void Solve(const solver::SolverOption& option, solver::Summary&
  // summary) = 0;
 public:
  virtual ~ADMM() { /* LOG(INFO) << "ADMM destructor"; */
  }

 protected:
  virtual bool IsConverge(const double tolerance, double& prev_delta,
                          double& primal_residual, double& dual_residual) = 0;

  virtual void SetPenaltyParameter() = 0;

  virtual void SetStepSize() = 0;

  virtual double ComputePrimalResidual() const = 0;

  virtual double ComputeDualResidual() const = 0;

  virtual void LogToStd(const size_t& iter, const double& prev_primal_residual,
                        const double& cur_primal_residual,
                        const double& prev_dual_residual,
                        const double& cur_dual_residual,
                        const double& time) const {
    if (iter < 1) {
      std::cout << "\n"
                << std::setw(11) << std::setfill(' ') << "Iter "
                << std::setw(16) << std::setfill(' ') << "PPR  "
                << std::setw(16) << std::setfill(' ') << "CPR  "
                << std::setw(16) << std::setfill(' ') << "PDR  "
                << std::setw(16) << std::setfill(' ') << "CDR  "
                << std::setw(16) << std::setfill(' ') << "Time ";
    } else {
      std::cout << std::setw(8) << std::setfill(' ') << iter << std::setw(5)
                << std::setfill(' ') << " " << std::setw(14)
                << std::setfill(' ') << prev_primal_residual << std::setw(5)
                << std::setfill(' ') << " " << std::setw(12)
                << std::setfill(' ') << cur_primal_residual << std::setw(5)
                << std::setfill(' ') << " " << std::setw(12)
                << std::setfill(' ') << prev_dual_residual << std::setw(5)
                << std::setfill(' ') << " " << std::setw(12)
                << std::setfill(' ') << cur_dual_residual << std::setw(2)
                << std::setfill(' ') << " " << std::setw(10)
                << std::setfill(' ') << time;
    }
    std::cout << "\n";
  }
};
}  // namespace DAGSfM

#endif