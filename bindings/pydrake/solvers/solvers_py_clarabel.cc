#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/bindings/pydrake/solvers/solvers_pybind.h"
#include "drake/solvers/clarabel_solver.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversClarabel(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  py::class_<ClarabelSolver, SolverInterface> cls(
      m, "ClarabelSolver", doc.ClarabelSolver.doc);
  cls.def(py::init<>(), doc.ClarabelSolver.ctor.doc)
      .def_static("id", &ClarabelSolver::id, doc.ClarabelSolver.id.doc);

  py::class_<ClarabelSolverDetails>(
      m, "ClarabelSolverDetails", doc.ClarabelSolverDetails.doc);
  AddValueInstantiation<ClarabelSolverDetails>(m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
