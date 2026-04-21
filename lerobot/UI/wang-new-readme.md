使用新的lerobot

python src/lerobot/scripts/lerobot_find_cameras.py
图片会保存在：mage capture finished. Images saved to outputs/captured_images
VideoCapture ctrl+f


ls /dev/ttyACM*
主臂 ttyACM0
从臂 ttyACM1
sudo chmod 777 /dev/ttyACM*


lerobot-calibrate \
--teleop.type=so100_leader \
--teleop.port=/dev/ttyACM0 \
--teleop.id=leader

lerobot-calibrate \
--robot.type=so100_follower \
--robot.port=/dev/ttyACM1 \
--robot.id=follower

lerobot-teleoperate \
--robot.type=so100_follower \
--robot.port=/dev/ttyACM1 \
--robot.cameras="{ top: {type: opencv, index_or_path: 2, width: 1280, height: 720, fps: 30, fourcc: MJPG},
wrist: {type: opencv, index_or_path: 0, width: 1280, height: 720, fps: 30, fourcc: MJPG}
}" \
--robot.id=follower \
--teleop.type=so100_leader \
--teleop.port=/dev/ttyACM0 \
--teleop.id=leader \
--display_data=true






lerobot-record \
       --robot.type=so100_follower \
       --robot.port=/dev/ttyACM1 \
       --robot.cameras='{
       top: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30, fourcc: MJPG}, 
       wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: MJPG}
       }' \
       --robot.id=follower \
       --teleop.type=so100_leader \
       --teleop.port=/dev/ttyACM0 \
       --teleop.id=leader \
       --dataset.repo_id=wang/newlerobotdataset \
       --dataset.single_task="Pick up the black packet and put it in the plastic basket" \
       --dataset.episode_time_s=15 \
       --dataset.reset_time_s=5 \
       --dataset.num_episodes=10 \
       --dataset.push_to_hub=false \
       --display_data=true




lerobot-replay \
    --robot.type=so100_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=follower \
    --dataset.repo_id=wang/newlerobotdataset \
    --dataset.episode=3



sudo apt-get install -y ffmpeg libavformat-dev libavcodec-dev  libavdevice-dev libavutil-dev libavfilter-dev libswscale-dev  libswresample-dev
conda install -c conda-forge ffmpeg=7 -y
export LD_PRELOAD=/home/wang/anaconda3/envs/robot/lib/libtiff.so.6



<!-- lerobot-dataset-viz     --repo-id wang/newlerobotdataset     --episode-index 3     --display-compressed-images False -->
 lerobot-train \
  --dataset.repo_id='/home/wang/.cache/huggingface/lerobot/wang/newlerobotdataset' \
  --policy.type=act \
  --policy.push_to_hub=false \
  --output_dir=outputs/train/act_new_so100 \
  --job_name=act_new_so100 \
  --policy.device=cuda \
  --wandb.enable=false 



  lerobot-record \
    --robot.type=so100_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.cameras='{
       top: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30, fourcc: MJPG}, 
       wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: MJPG}
    }' \
    --robot.id=follower \
    --teleop.type=so100_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader \
    --policy.path=outputs/train/act_new_so100/checkpoints/030000/pretrained_model \
    --dataset.repo_id=wang/eval_newlerobotdataset \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=3 \
    --dataset.num_episodes=1 \
    --dataset.single_task="Pick up the black packet and put it in the plastic basket" \
    --dataset.push_to_hub=false \
    --display_data=true




