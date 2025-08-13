function tmc_wrs_gazebo_worlds_setup {
        local model_path=/home/laowang/workspace/tutorial5_ws/install/share/tmc_wrs_gazebo_worlds/models
    
    if [ -z $GAZEBO_MODEL_PATH ]
    then
        export GAZEBO_MODEL_PATH=$model_path
    else
        export GAZEBO_MODEL_PATH=$model_path:$GAZEBO_MODEL_PATH
    fi
}
tmc_wrs_gazebo_worlds_setup
