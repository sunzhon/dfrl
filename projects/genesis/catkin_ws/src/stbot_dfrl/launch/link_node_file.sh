#! /bin/sh

current_path=$(dirname $(readlink -f $0))
exec_node="$current_path/../stbot_dfrl_node"
build_node="$current_path/../../../devel/lib/stbot_dfrl/stbot_dfrl_node"
if [ ! -f "$exec_node" ];then
    ln -s $build_node $exec_node
fi

