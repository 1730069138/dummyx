#实际数据推理：
#1.一个终端连接远程服务器：
ssh -L 8000:localhost:8000 jun@100.64.142.55
cd ~/VLA/openpi
source .venv/bin/activate
CUDA_VISIBLE_DEVICES=5 uv run scripts/serve_policy.py --port 8000 policy:checkpoint --policy.config pi0_dummy_real_lora --policy.dir /home/jun/VLA/openpi/checkpoints/pi0_dummy_real_lora/real_v2/10000

#2.另一个终端启动本地推理脚本：
conda activate dummyx_vla
cd ~/dummyx/webgui
python3 deploy_real_arm_vanilla.py 

#实际数据集采集：
conda activate dummyx_vla
cd ~/dummyx/webgui
#启动gui自动校准先
python3 gui.py
#启动采集脚本键盘控制采集
python3 collect_data.py
#采集完之后发送到远程服务器去转换
conda activate dummyx_vla
cd ~/dummyx/webgui
python3 convert_real_to_lerobot.py
