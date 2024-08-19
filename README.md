    git submodule update --init --recursive 

    mkdir build && cd build
    cmake ..
    make

    ./admittance_control_example ../config/admittance_control_example.yaml                   
    ./dual_arm_admittance_control_example  ../config/dual_arm_admittance_control_example.yaml