#!/usr/bin/env python3
import subprocess
import time
import os
import sys

def run_command(cmd, description):
    print(f"\n{'='*60}")
    print(f"🚀 開始執行: {description}")
    print(f"命令: {' '.join(cmd)}")
    print(f"{'='*60}\n")
    
    try:
        # 執行命令，將輸出直接顯示在終端機
        result = subprocess.run(cmd, check=True)
        print(f"\n✅ 完成: {description}\n")
        return True
    except subprocess.CalledProcessError as e:
        print(f"\n❌ 錯誤: {description} 執行失敗！錯誤碼: {e.returncode}\n")
        return False
    except KeyboardInterrupt:
        print(f"\n🛑 使用者中斷: {description}\n")
        return False

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 讓使用者輸入左轉角度
    try:
        user_input = input("請輸入抓取後左轉的角度 (度, 預設 180.0): ")
        turn_angle = float(user_input) if user_input.strip() else 180.0
    except ValueError:
        print("輸入無效，使用預設值 180.0")
        turn_angle = 180.0
    
    # 讓使用者輸入第一次直走的距離
    try:
        user_dist_input = input("請輸入第一次直走的距離 (CM, 預設 130.0): ")
        move_dist = float(user_dist_input) if user_dist_input.strip() else 130.0
    except ValueError:
        print("輸入無效，使用預設值 130.0")
        move_dist = 130.0
        
    # 讓使用者輸入第二次直走(回程)的距離
    try:
        user_dist2_input = input("請輸入抓取並轉向後直走的距離 (公尺, 預設 1.2): ")
        move_dist2 = float(user_dist2_input) if user_dist2_input.strip() else 1.2
    except ValueError:
        print("輸入無效，使用預設值 1.2")
        move_dist2 = 1.2
    
    # 步驟 1: 使用 test_lidar_orientation.py 的直行前進至橋中心 (GO) 模式
    cmd_move_1 = ["python3", os.path.join(script_dir, "test_lidar_orientation.py"), "--auto-go", str(move_dist)]
    if not run_command(cmd_move_1, f"光達導航: 直行前進 {move_dist} CM"):
        print("❌ 終止後續任務")
        sys.exit(1)
        
    time.sleep(1) # 暫停 1 秒確保系統穩定
        
    # 步驟 2: 讓 camera and grab.py 判斷被抓去抓取 (會自動切換為對齊模式與夾取序列)
    camera_grab_path = os.path.join(script_dir, "workspaces/src/arm_ik/arm_ik/camera and grab.py")
    cmd_grab = ["python3", camera_grab_path]
    if not run_command(cmd_grab, "相機判斷並抓取"):
        print("❌ 終止後續任務")
        sys.exit(1)
        
    time.sleep(1)
        
    # 步驟 3: 左轉指定角度
    # 使用新建立的 turn_angle.py 來進行旋轉
    cmd_turn = ["python3", os.path.join(script_dir, "turn_angle.py"), str(turn_angle)]
    if not run_command(cmd_turn, f"左轉 {turn_angle} 度"):
        print("❌ 終止後續任務")
        sys.exit(1)
        
    time.sleep(1)
        
    # 步驟 4: 直走指定距離
    cmd_move_2 = ["python3", os.path.join(script_dir, "move_distance.py"), str(move_dist2)]
    if not run_command(cmd_move_2, f"直走 {move_dist2 * 100} CM"):
        print("❌ 終止後續任務")
        sys.exit(1)
        
    time.sleep(1)
        
    # 步驟 5: 手臂復歸至點位 1
    cmd_reset_arm = ["python3", camera_grab_path, "1"]
    if not run_command(cmd_reset_arm, "手臂復歸至點位 1"):
        print("❌ 終止後續任務")
        sys.exit(1)
        
    print("\n🎉 所有指令執行完畢！")

if __name__ == '__main__':
    main()
