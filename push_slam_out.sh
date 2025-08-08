#!/usr/bin/env bash

# Check if an argument was provided
if [ $# -lt 1 ]; then
  echo "Usage: $0 <trialname>"
  exit 1
fi

TRIAL=$1

ansible-playbook ansible/push/push.yml -e "file_paths=['orbslam/out/${TRIAL}_cam_traj.txt','orbslam/out/${TRIAL}_kf_traj.txt']"
