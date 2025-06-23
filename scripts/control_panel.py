import time  
import json
import subprocess
from openai import OpenAI

client = OpenAI(
    api_key="sk-977ce52689fc4e5db0d5af18d6db50bf",
    base_url="https://api.deepseek.com"
)

def print_menu():
    print("\n请选择功能：")
    print("1. A*算法（单个机器人）")
    print("2. dijkstra算法（单个机器人）")
    print("3. 图算法（单个机器人）")
    print("4. 详细指令")
    print_robot()

def print_robot():
    print(" ┌───────┐ ")
    print(" │ [● ●] │")
    print(" │   ▽   │")
    print(" └─┬─┬─┬─┘")
    print("   │ │ │  ")

def get_valid_input(prompt, valid_choices):
    while True:
        choice = input(prompt).strip()
        if choice in valid_choices:
            return choice
        print("输入无效，请重新输入。")

def start_ros_project(x_pose=0, y_pose=0):
    """依次启动3个gnome-terminal，每个间隔10秒，可自定义初始坐标。"""
    setup_cmd = ". install/setup.bash && export TURTLEBOT3_MODEL=burger && "
    cmds = [
        f"{setup_cmd}ros2 launch delivery_bot obstacle_course.launch.py x_pose:={x_pose} y_pose:={y_pose}",
        f"{setup_cmd}ros2 launch delivery_bot cartographer.launch.py use_sim_time:=true",
        f"{setup_cmd}ros2 launch delivery_bot dijkstra.launch.py"#hamburger_walker.launch.py/astar.launch.py
    ]
    for i, cmd in enumerate(cmds):
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', cmd])
        if i < len(cmds) - 1:
            time.sleep(10)  # 启动完一个后等待10秒再启动下一个

def parse_instruction(user_content):
    try:
        response = client.chat.completions.create(
            model="deepseek-chat",
            messages=[
                {
                    "role": "system",
                    "content": """我有一项专注于机器人送餐的项目,我的需求是输入一段自然语言指令,
                    例如:把***送到***。我需要你提取这一段语言输入指令,输出物体以及目的地,使用英文。以字符串的形式输出。例如:
                    把饮用水送到A302，
                    你需要返回：
                    {
                        "object": "water", 
                        "destination": "A302"
                    },
                    以字符串的形式返回，只返回大括号以及其内部内容，不要有其他任何内容。
                    """
                },
                {"role": "user", "content": user_content}
            ],
            stream=False,
            temperature=1.0,
        )
        content = response.choices[0].message.content
        result = json.loads(content)
        return result
    except Exception:
        return None

def detail_instruction_interface():
    while True:
        print("\n—— 详细指令界面 ——")
        print("请输入您的指令（输入b返回上一级）：")
        user_input = input(">> ").strip()
        if user_input.lower() == 'b':
            return
        parsed = parse_instruction(user_input)
        if parsed and "object" in parsed and "destination" in parsed:
            print(f"收到！我将送{parsed['object']}到{parsed['destination']}")
            start_ros_project(2, 0)  
            break
        else:
            print("抱歉！我无法理解你的命令。")

def main():
    while True:
        print_menu()
        choice = get_valid_input("请选择功能 (1-4)：", ['1', '2', '3', '4'])
        if choice == '1':
            print("收到！机器人已在场景中运行")
            start_ros_project(0, 2)   
            break
        elif choice == '2':
            print("收到！机器人已在场景中运行")
            start_ros_project(0, 0)    
            break
        elif choice == '3':
            print("收到！机器人已在场景中运行")
            start_ros_project(3, -2)   
            break
        elif choice == '4':
            detail_instruction_interface()
            break

if __name__ == "__main__":
    main()

