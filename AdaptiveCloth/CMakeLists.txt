add_library(
    adaptive_cloth_lib Application.cpp auglag.cpp bah.cpp bvh.cpp cloth.cpp collision.cpp collisionutil.cpp conf.cpp constraint.cpp dde.cpp 
    display.cpp 
    dynamicremesh.cpp geometry.cpp handle.cpp io.cpp lsnewton.cpp mesh.cpp morph.cpp mot_parser.cpp nearobs.cpp obstacle.cpp physics.cpp plasticity.cpp popfilter.cpp proximity.cpp remesh.cpp separate.cpp separateobs.cpp Simulation.cpp 
    StripSimulation.cpp
    spline.cpp strainlimiting.cpp taucs.cpp tensormax.cpp transformation.cpp util.cpp vectors.cpp)