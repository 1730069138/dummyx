![3](images/webgui.png)

修改arm_server.py后
# 重新加载配置
sudo systemctl daemon-reload
# 重启服务
sudo systemctl restart arm_server.service
# 设置开机自启
sudo systemctl enable arm_server.service
# 立即手动启动测试
sudo systemctl start arm_server.service
# 查看运行状态
sudo systemctl status arm_server.service
# 查看运行日志信息
sudo journalctl -u arm_server.service -f
# 运行demo
sudo python3 demo.py --keep-alive
