import os

def main():
    print("🚀 正在当前终端窗口内，追加 4 个核心服务标签页...")

    # 核心魔法：只用 --tab，不用 --window，系统就会默认在当前终端增加标签页！
    # 使用 -e 参数而不是 --，这样可以在一行命令里连续开多个 tab 而不会被截断
    cmd = (
        "gnome-terminal "
        "--tab --title='CarState' -e \"bash -c 'source install/setup.bash && python3 car_state.py; exec bash'\" "
        "--tab --title='Image' -e \"bash -c 'source install/setup.bash && python3 image_transport.py; exec bash'\" "
        "--tab --title='Map' -e \"bash -c 'source install/setup.bash && python3 upload_map.py; exec bash'\" "
        "--tab --title='Zuobiao' -e \"bash -c 'source install/setup.bash && python3 zuobiao.py; exec bash'\""
    )

    # 发送系统指令
    os.system(cmd)
    
    print("✅ 搞定！请看终端上方，4 个独立的日志标签页已经出现啦。")
    print("💡 当前这个启动页面你可以留着，也可以直接输入 exit 关掉，不影响那 4 个程序。")

if __name__ == '__main__':
    main()
