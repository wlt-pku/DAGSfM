set(FOLDER_NAME "solver")

COLMAP_ADD_SOURCES(
    admm.h
    constrained_l1_solver.h constrained_l1_solver.cpp
    l1_solver.h
    qp_solver.h qp_solver.cpp
    summary.h
)