import pyrealsense2 as rs
import cv2
import numpy as np
import time

def main():
    print("🔍 正在扫描系统中的 RealSense 相机...")
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        print("❌ 没有检测到任何 RealSense 相机！请检查 USB 连线或 Hub 供电。")
        return
        
    print(f"✅ 共检测到 {len(devices)} 台相机：")
    
    pipelines = {}
    
    # 遍历所有设备，提取信息并尝试启动数据流
    for dev in devices:
        name = dev.get_info(rs.camera_info.name)
        sn = dev.get_info(rs.camera_info.serial_number)
        usb_type = dev.get_info(rs.camera_info.usb_type_descriptor)
        
        print(f"  - 型号: {name: <20} | SN: {sn: <15} | USB版本: {usb_type}")
        
        if "2.1" in usb_type:
            print(f"  ⚠️ 警告: 相机 {sn} 运行在 USB 2.0 模式下，可能会导致带宽不足或掉帧！")

        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(sn)
        
        # 使用较低分辨率，防止同时开启 3 个相机时 USB 带宽溢出
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        try:
            pipe.start(cfg)
            pipelines[sn] = pipe
        except Exception as e:
            print(f"  ❌ 无法启动相机 {sn} 的视频流: {e}")

    if not pipelines:
        print("❌ 所有相机启动失败，请检查是否被其他程序占用。")
        return

    print("\n=======================================================")
    print("📸 正在打开预览窗口...")
    print("💡 标定步骤：")
    print("   1. 请用手依次在每个镜头前晃动。")
    print("   2. 观察哪个窗口出现了你的手。")
    print("   3. 记下该窗口上显示的 SN 码，用于后续主脚本配置。")
    print("   (在任意预览窗口选中时，按 'q' 键退出)")
    print("=======================================================\n")
    
    try:
        while True:
            # 使用 poll_for_frames 而不是 wait，防止某个相机卡死导致整个画面冻结
            for sn, pipe in pipelines.items():
                success, frames = pipe.try_wait_for_frames(timeout_ms=50)
                if not success:
                    continue
                    
                color_frame = frames.get_color_frame()
                if color_frame:
                    img = np.asanyarray(color_frame.get_data())
                    
                    # 在画面上打上醒目的文字
                    cv2.putText(img, f"SN: {sn}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                    cv2.imshow(f"Camera - SN: {sn}", img)
                    
            # 监听按键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        print("正在关闭相机流...")
        for pipe in pipelines.values():
            pipe.stop()
        cv2.destroyAllWindows()
        print("退出成功。")

if __name__ == "__main__":
    main()