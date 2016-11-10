#!/usr/bin/env bash

# Handle keyboard interrupt
trap " exit " INT

PROGNAME=$(basename $0)

# Default argument list
robot_list=( spider9 spider13 spider17 gecko7 gecko12 gecko17 snake5 snake7 snake9 babyA babyB babyC )

config=rlpower.cfg
gz_command=gzserver
load_controller=
manager=single_robot_manager.py
no_experiments=10
output=output
restore=restore
world=gait-learning.world

function error_exit() {
    echo "${PROGNAME}: ${1:-"Unknown Error"}" 1 >&2
    exit 1
}

function help() {
    echo "Usage: ${PROGNAME} [args...]"
    echo "Arguments:"
    echo " -c | --config          Path to a robot brain config file"
    echo " -g | --gzcommand       Gazebo command [gzserver|gazebo] Default: gzserver"
    echo " -h | --help            Help page"
    echo " -l | --load            Path to a robot controller file"
    echo " -m | --manager         Name of a script that controls robots and the environment"
    echo " -n | --no-experiments  Number of experiment repetitions"
    echo " -o | --output          Name of a output directory"
    echo " -r | --restore         Name of a restore directory"
    echo " -w | --world           Name of a world file"

    exit 0
}

function main() {

    # Read out the argument list
    if [ $# -gt 0 ]; then
        while [ "$1" != "" ]
        do
            local argument="$1";
            shift
            local parameter="$1";
            shift
            case ${argument} in
                -c | --config) local config=${parameter} ;;
                -g | --gzcommand) local gz_command=${parameter} ;;
                -h | --help) help ;;
                -l | --load) local load_controller=${parameter} ;;
                -m | --manager) local manager=${parameter} ;;
                -n | --no-experiments) local no_experiments=${parameter} ;;
                -o | --output) local output=${parameter} ;;
                -r | --restore) local restore=${parameter} ;;
                -w | --world) local world=${parameter} ;;
                *) error_exit "${LINENO}: In ${FUNCNAME}() unknown argument ${argument}." ;;
            esac
        done
    fi

    # For each name in 'robot_list' run 'no_experiments' experiments
    for index in ${!robot_list[*]}
    do
        for (( i = 1; i <= ${no_experiments}; ++ i ))
        do
            python start.py \
                --load-controller ${load_controller} \
                --manager ${manager} \
                --world ${world} \
                --output ${output} \
                --restore ${restore} \
                --robot-name robots/${robot_list[$index]} \
                --experiment-round ${i} \
                --brain-conf-path ${config} \
                --gazebo-cmd ${gz_command}
        done
    done

    exit 0
}

main $@
