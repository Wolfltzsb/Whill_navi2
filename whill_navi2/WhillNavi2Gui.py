import tkinter as tk
import yaml, os, time, signal
from tkinter import scrolledtext, messagebox, font, filedialog
import datetime, subprocess, psutil
import rclpy.duration
from whill_navi2.modules.ros2_launch_utils import DataPath, get_file_path, get_param_dict, split_bag
from rclpy.node import Node
import rclpy, shutil
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from nmea_msgs.msg import Sentence


# def set_topmost(window : tk.Wm | None):
#     try:
#         window.attributes('-topmost', True)
#         window.update()
#     except:
#         pass

def on_main_close():
    messagebox.showwarning("Some tasks is not over!")

def terminate_all_ros(pids : list | None):
    exe_names = []
    # print(pids)
    # if pids == None:
    #     return
# try:
#     try:
    subprocess.run(["pkill", "--full", "ros2 launch"])
    # except:
        # pass
    time.sleep(1.0)
        # parent = psutil.Process(pid)
        # children = parent.children(recursive=True)
    for proc in psutil.process_iter(['exe', 'cmdline']):
        exe_name = str(proc.info["exe"])
        cmdline = proc.info["cmdline"]
        print(proc.info)
        for content in cmdline:
            if "WhillNavi2Gui" in content:
                continue
            elif "whill_navi2_node" in content:
                exe_names.append(content)
            elif "lidar_stop_node" in content:
                exe_names.append(content)
        
        # type(exe_name)
        if "/opt/ros" in exe_name and "lib" in exe_name:
            print(exe_name)
            exe_names.append(exe_name)
        elif os.environ["HOME"] in exe_name and "install" in exe_name and "lib" in exe_name:
            print(exe_name)
            exe_names.append(exe_name)

        # 获取子进程的pid
        # child_pids = [child.pid for child in children]
        # 终止所有子进程
        # for child in children:
        #     child.terminate()
        #     child.kill()
        
        # 等待子进程结束
        # gone, alive = psutil.wait_procs(children, timeout=3)
        
        # 如果还有进程存活，强制结束它们
        # for p in alive:
        #     p.kill()
            
    # except psutil.NoSuchProcess:
    #     pass

    # 关闭gnome-terminal窗口
    for exe_name in exe_names:
        subprocess.run(["pkill", "--full", exe_name])

    pids.clear()

def data_gather():
    def on_make_dir_changed(*args):
        make_dir_dict["place_dir"] = place_name_entry_var.get()
        make_dir_dict["bag_name"] = bag_name_entry_var.get()
        with open(make_dir_yaml_path, "w+") as f:
            write_data = {}
            write_data["make_dir_node"] = {}
            write_data["make_dir_node"]["ros__parameters"] = make_dir_dict
            yaml.safe_dump(write_data, f)
        
    def launch_data_gather(*args):
        def velodyne_callback(msg : PointCloud2):
            if msg.header.frame_id:
                velodyne_state_checkbox.config(fg="green")
        
        def zed_camera_callback(msg : Imu):
            if msg.header.frame_id:
                # print(msg.header.frame_id)
                zed_camera_state_checkbox.config(fg="green")
        
        def ublox_callback(msg : Sentence):
            if msg.header.frame_id:
                # print(msg.header.frame_id)
                ublox_state_checkbox.config(fg="green")
        
        def adis_imu_callback(msg : Imu):
            if msg.header.frame_id:
                # print(msg.header.frame_id)
                adis_imu_state_checkbox.config(fg="green")
        
        def ylm20_callback(msg : PointCloud2):
            if msg.header.frame_id:
                # print(msg.header.frame_id)
                ylm20_state_checkbox.config(fg="green")
                
        bag_file_path_var.set(os.path.join(DataPath().bag_dir_path, current_datetime.strftime("%H-%M-%S") + "_" + bag_name_entry_var.get()))
        command = "gnome-terminal --tab --name=DataGather --title=DataGather -- ros2 launch whill_navi2 data_gather.launch.py {}".format(
            " ".join([
                "bag_file_path:={}".format(bag_file_path_var.get()),
                "use_adis_imu:={}".format(adis_imu_state_var.get()),
                # "use_wit_imu:={}".format(wit_imu_state_var.get()),
                "use_velodyne:={}".format(velodyne_state_var.get()),
                "use_ublox:={}".format(ublox_state_var.get()),
                # "use_hokuyo:={}".format(hokuyo_state_var.get()),
                "use_zed_camera:={}".format(zed_camera_state_var.get()),
                "use_m20_lidar:={}".format(ylm20_state_var.get())
            ])
        )
        # print(command)
        process_pids.append(subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True).pid)
        rclpy.init()
        
        check_sensor_topic_node = Node("check_sensor_topic_node")
        if velodyne_state_var.get():
            velodyne_sub = check_sensor_topic_node.create_subscription(PointCloud2, "velodyne_points", velodyne_callback, 2)
        if zed_camera_state_var.get():
            zed_camera_sub = check_sensor_topic_node.create_subscription(Imu, "zed2i/zed_node/imu/data", zed_camera_callback, 2)
        if ublox_state_var.get():
            ublox_sub = check_sensor_topic_node.create_subscription(Sentence, "ublox/nmea", ublox_callback, 2)
        if adis_imu_state_var.get():
            adis_imu_sub = check_sensor_topic_node.create_subscription(Imu, "adis/imu/data", adis_imu_callback, 2)
        if ylm20_state_var.get():
            ylm20_sub = check_sensor_topic_node.create_subscription(PointCloud2, "lumotive_ros/pointcloud", ylm20_callback, 2)
        
        for index in range(20):
            check_sensor_topic_node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0, nanoseconds=int(1e8)))
            rclpy.spin_once(node=check_sensor_topic_node)
        
        rclpy.shutdown()
        check_sensor_topic_node.destroy_node()

        # check_gnss_command = "gnome-terminal --tab --name=CheckGNSS --title=CheckGnss -- ros2 topic echo /gnss/fix"
        # subprocess.Popen(check_gnss_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
        # check_adis_command = "gnome-terminal --tab --name=CheckAdis --title=CheckAdis -- ros2 topic echo /adis/imu/data"
        # subprocess.Popen(check_adis_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
             
    def launch_geopoints_gather():
        command = "gnome-terminal --tab --name=GeopointsGather --title=GeopointsGather -- ros2 launch whill_navi2 draw_geopoints_gepro.launch.py"
        subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
        
    def on_sensor_changed():
        if velodyne_state_var.get():
            velodyne_state_checkbox.config(fg="red")
        else:
            velodyne_state_checkbox.config(fg="gray")
        if zed_camera_state_var.get():
            zed_camera_state_checkbox.config(fg="red")
        else:
            zed_camera_state_checkbox.config(fg="gray")
        if adis_imu_state_var.get():
            adis_imu_state_checkbox.config(fg="red")
        else:
            adis_imu_state_checkbox.config(fg="gray")
        if ylm20_state_var.get():
            ylm20_state_checkbox.config(fg="red")
        else:
            ylm20_state_checkbox.config(fg="gray")
        if ublox_state_var.get():
            ublox_state_checkbox.config(fg="red")
        else:
            ublox_state_checkbox.config(fg="gray")

    def launch_whill():
        command = "gnome-terminal --tab --name=Whill --title=Whill -- ros2 launch whill_navi2 kuaro_whill.launch.py"
        process_pids.append(subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True).pid)
    
    data_gather_menu = tk.Toplevel(root)
    data_gather_menu.title("DataGather")
    data_gather_menu.geometry("450x200")
    process_pids = []

    data_gather_menu.grab_set()
    
    # Path configuration
    path_configuration_frame = tk.Frame(data_gather_menu)
    path_configuration_frame.grid(row=0, column=0, sticky="w")
    place_name_title = tk.Label(path_configuration_frame, text="PlaceName", font=bold_font)
    place_name_title.grid(row=0, column=0, padx=10, sticky="w")
    place_name_entry_var = tk.StringVar(data_gather_menu, make_dir_dict["place_dir"], "place_name")
    place_name_entry_var.trace_add("write", on_make_dir_changed)
    place_name_entry = tk.Entry(path_configuration_frame, font=normal_font, width=10, textvariable=place_name_entry_var)
    place_name_entry.grid(row=1, column=0, padx=10, sticky="w")

    date_name_title = tk.Label(path_configuration_frame, text="DateName", font=bold_font)
    date_name_title.grid(row=0, column=1, padx=10, sticky="w")
    
    bag_name_title = tk.Label(path_configuration_frame, text="BagName", font=bold_font)
    bag_name_title.grid(row=0, column=2, padx=10, sticky="w")
    bag_name_entry_var = tk.StringVar(data_gather_menu, make_dir_dict["bag_name"])
    bag_name_entry_var.trace_add("write", on_make_dir_changed)
    bag_name_entry = tk.Entry(path_configuration_frame, textvariable=bag_name_entry_var, font=normal_font, width=15)
    bag_name_entry.grid(row=1, column=2, padx=10, sticky="w")
    current_datetime = datetime.datetime.now()
    datetime_str = current_datetime.strftime("%Y-%m-%d")
    date_name_entry_var = tk.StringVar(data_gather_menu, datetime_str, "date_name")
    make_dir_dict["date_dir"] = date_name_entry_var.get()
    with open(make_dir_yaml_path, "w+") as f:
        write_data = {}
        write_data["make_dir_node"] = {}
        write_data["make_dir_node"]["ros__parameters"] = make_dir_dict
        yaml.safe_dump(write_data, f)
    # date_name_entry_var.trace_add("write", on_date_name_changed)
    date_name_entry = tk.Label(path_configuration_frame, font=normal_font, textvariable=date_name_entry_var)
    date_name_entry.grid(row=1, column=1, padx=10, sticky="w")

    # Sensor configuration
    sensor_configuration_frame = tk.Frame(data_gather_menu)
    sensor_configuration_frame.grid(row=1, column=0, pady=2, sticky="nw")
    
    # sensor_label = tk.Label(sensor_configuration_frame, text="------------  Sensor  ------------", font=bold_font)
    # sensor_label.grid(row=0, column=0, columnspan=4, pady=10, sticky="w")
    reset_sensor_configuration_button = tk.Button(sensor_configuration_frame, text="Reset", command=lambda: [adis_imu_state_var.set(False), zed_camera_state_var.set(False), velodyne_state_var.set(False), ylm20_state_var.set(False), ublox_state_var.set(False), adis_imu_state_checkbox.config(fg="gray"), zed_camera_state_checkbox.config(fg="gray"), velodyne_state_checkbox.config(fg="gray"), ylm20_state_checkbox.config(fg="gray"), ublox_state_checkbox.config(fg="gray")])
    reset_sensor_configuration_button.grid(row=4, column=0, pady=10, sticky="w")
    terminate_button = tk.Button(sensor_configuration_frame, text="TerminateALL", command=lambda : terminate_all_ros(process_pids))
    terminate_button.grid(row=4, column=1, pady=10, sticky="w")
    reset_sensor_configuration_button.grid(row=4, column=0, columnspan=2, padx=10, sticky="w")
    velodyne_state_var = tk.BooleanVar(data_gather_menu, False)
    velodyne_state_checkbox = tk.Checkbutton(sensor_configuration_frame, text="Velodyne", variable=velodyne_state_var, font=normal_font, fg="gray", command=on_sensor_changed)
    velodyne_state_checkbox.grid(row=1, column=0, padx=5, sticky="w")
    zed_camera_state_var = tk.BooleanVar(data_gather_menu, False)
    zed_camera_state_checkbox = tk.Checkbutton(sensor_configuration_frame, text="Zed2i Camera", variable=zed_camera_state_var, font=normal_font, fg="gray", command=on_sensor_changed)
    zed_camera_state_checkbox.grid(row=1, column=1, padx=5, sticky="w")
    adis_imu_state_var = tk.BooleanVar(data_gather_menu, False)
    adis_imu_state_checkbox = tk.Checkbutton(sensor_configuration_frame, text="Adis16465", variable=adis_imu_state_var, font=normal_font, fg="gray", command=on_sensor_changed)
    adis_imu_state_checkbox.grid(row=2, column=0, padx=5, sticky="w")
    ublox_state_var = tk.BooleanVar(data_gather_menu, False)
    ublox_state_checkbox = tk.Checkbutton(sensor_configuration_frame, text="Ublox", variable=ublox_state_var, font=normal_font, fg="gray", command=on_sensor_changed)
    ublox_state_checkbox.grid(row=2, column=1, padx=5, sticky="w")
    ylm20_state_var = tk.BooleanVar(data_gather_menu, False)
    ylm20_state_checkbox = tk.Checkbutton(sensor_configuration_frame, text="Ylm20", variable=ylm20_state_var, font=normal_font, fg="gray", command=on_sensor_changed)
    ylm20_state_checkbox.grid(row=3, column=0, padx=5, sticky="w")
    launch_whill_button = tk.Button(sensor_configuration_frame, text="LaunchWhill", command=launch_whill)
    launch_whill_button.grid(row=3, column=1, padx=5, sticky="w")

    data_gather_button_frame = tk.Frame(data_gather_menu)
    data_gather_button_frame.grid(row=2, column=0, columnspan=3, padx=10, pady=10)
    data_gather_start_button = tk.Button(data_gather_button_frame, text="LaunchGather", command=launch_data_gather)
    data_gather_start_button.grid(row=0, column=0, sticky="n")
    geopoints_gather_button = tk.Button(data_gather_button_frame, text="LaunchGeopoints", command=launch_geopoints_gather)
    geopoints_gather_button.grid(row=0, column=1, padx=15, sticky="n")
    
    adjust_window_size(data_gather_menu)
    # set_topmost(data_gather_menu)
    data_gather_menu.mainloop()

def glim_offline():
    # command = "gnome-terminal --tab --name=GlimOffline --title=GlimOffline -- ros2 launch whill_navi2 glim_offline.launch.py bag_file_path:={}".format(bag_file_path_var.get())
    # subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
    # root_log_box.insert(tk.END, command + "\n")
    # root_log_box.see(tk.END)
    
    def slice_checkbox_callback():
        if slice_checkbox_var.get():
            slice_pieces_entry.config(state="normal")
            slice_unit_title.config(state="normal")
            command_genrate_button.config(state="normal")
            launch_button_var.set("Launch Slice")
        else:
            slice_pieces_entry.config(state="disabled")
            slice_unit_title.config(state="disabled")
            command_genrate_button.config(state="disabled")
            launch_button_var.set("Launch Full")

        slice_pieces_entry_var.set(1)
        command_scrolledtext.delete("1.0", tk.END)
        command_scrolledtext.see("1.0")
        command_scrolledtext.mark_set(tk.INSERT, "1.0")
        command_list.clear()
        slice_number = 0
    
    def generate_command():
        slice_number = slice_pieces_entry_var.get()
        bag_duration = bag_duration_var.get() / slice_pieces_entry_var.get()
        real_bag_duration = bag_duration / bag_rate_var.get()

        for index in range(slice_pieces_entry_var.get()):
            bag_start_offset = bag_duration * index
            command = "gnome-terminal --tab --name=GlimOffline{} --title=GlimOffline{} -- ros2 launch whill_navi2 glim_offline.launch.py bag_file_path:={} bag_rate:={} bag_start_offset:={} bag_play_duration:={} glim_dump_path:={} glim_waypoint_path:={} slice_bag:={} namespace:={}".format(
                index, index, bag_file_path_var.get(), bag_rate_var.get(),
                bag_start_offset, real_bag_duration, 
                os.path.join(DataPath().pcd_map_dir_path, str(index) + "_" + "glim_dump"),
                os.path.join(DataPath().waypoint_dir_path, str(index) + "_" + "glim_waypoint.yaml"),
                str(slice_checkbox_var.get()), str("glim_slice_{}".format(index))
            )
            # print(command)
            command_list.append(command)
            command_scrolledtext.insert(tk.END, "Command{}: ".format(index + 1) + command + "\n")

    def on_line_click(event):
        """单击时选中整行"""
        # 获取鼠标点击位置的行号
        cursor_index = command_scrolledtext.index(tk.CURRENT)  # 当前光标位置
        line_number = cursor_index.split('.')[0]
        # 清除之前的高亮
        command_scrolledtext.tag_remove("highlight", "1.0", tk.END)
        # 高亮当前行，包括空白部分
        command_scrolledtext.tag_add("highlight", f"{line_number}.0", f"{int(line_number) + 1}.0")

    def launch_glim_offline():
        if slice_checkbox_var.get():
            try:
                start_index = command_scrolledtext.tag_ranges("highlight")[0]  # 高亮起点
                line_number = start_index.string.split('.')[0]               
                command_scrolledtext.delete(f"{line_number}.0", f"{line_number}.0 lineend+1c")
                command = command_list.pop(int(line_number) - 1)
                subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
            except IndexError:
                return
        else:
            command = "gnome-terminal --tab --name=GlimOffline --title=GlimOffline -- ros2 launch whill_navi2 glim_offline.launch.py bag_file_path:={}".format(bag_file_path_var.get())
            subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)

    def bag_file_glim():
        bag_file_path = filedialog.askdirectory(parent=glim_offline_menu, initialdir=DataPath().bag_dir_path)
        bag_file_path_var.set(bag_file_path)
        splited_path_list = os.path.split(bag_file_path)[0].split("/")
        if len(splited_path_list) >= 7:
            make_dir_dict["place_dir"] = splited_path_list[5]
            make_dir_dict["date_dir"] = splited_path_list[6]
            make_dir_dict["bag_name"] = os.path.split(bag_file_path)[1]
        
        with open(make_dir_yaml_path, "w+") as f:
            write_data = {}
            write_data["make_dir_node"] = {}
            write_data["make_dir_node"]["ros__parameters"] = make_dir_dict
            yaml.safe_dump(write_data, f)
        
        if os.path.exists(os.path.join(bag_file_path, "metadata.yaml")):
            with open(os.path.join(bag_file_path, "metadata.yaml"), "r") as f:
                bag_duration_var.set(yaml.load(f, Loader=yaml.FullLoader)["rosbag2_bagfile_information"]["duration"]["nanoseconds"] * 1e-9)
        
        reset_bag_file_path()
    
    def launch_viewwer():
        if slice_checkbox_var.get():
            command = "gnome-terminal --tab --name=GlimOfflineViewer --title=GlimOfflineViewer -- ros2 launch whill_navi2 glim_offline_viewer.launch.py"
            subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)

    glim_offline_menu = tk.Toplevel(root)
    glim_offline_menu.title("GlimOffline")
    glim_offline_menu.geometry("400x400")
    command_list = []
    
    glim_offline_menu.grab_set()
    
    bag_configuration_frame = tk.Frame(glim_offline_menu)
    bag_configuration_frame.grid(row=0, column=0, padx=5, pady=5)
    bag_duration_title = tk.Label(bag_configuration_frame, text="BagDuration: ", font=normal_font)
    bag_duration_title.grid(row=0, column=0, padx=5, pady=5)
    bag_duration_var = tk.DoubleVar(glim_offline_menu, 0.0, "bag_duration")
    for paths, dirs, files in os.walk(bag_file_path_var.get()):
        for file in files:
            if ".yaml" in file:
                with open(os.path.join(bag_file_path_var.get(), file), "r") as f:
                    bag_duration_var.set(yaml.load(f, Loader=yaml.FullLoader)["rosbag2_bagfile_information"]["duration"]["nanoseconds"] * 1e-9)
                break
    
    bag_duration_label = tk.Label(bag_configuration_frame, font=normal_font, textvariable=bag_duration_var)
    bag_duration_label.grid(row=0, column=1, padx=5, pady=5)
    bag_rate_title = tk.Label(bag_configuration_frame, text="BagRate: ", font=normal_font)
    bag_rate_title.grid(row=1, column=0, padx=5, pady=5)
    bag_rate_var = tk.DoubleVar(glim_offline_menu, 20.0, "bag_rate")
    bag_rate_entry = tk.Entry(bag_configuration_frame, font=normal_font, textvariable=bag_rate_var, width=5)
    bag_rate_entry.grid(row=1, column=1, padx=5, pady=5)
    select_bag_button = tk.Button(bag_configuration_frame, text="SelectBag", command=bag_file_glim)
    select_bag_button.grid(row=0, column=2, padx=5, pady=5)
    
    slice_option_frame = tk.Frame(glim_offline_menu)
    slice_option_frame.grid(row=1, column=0, padx=5, pady=5)
    slice_checkbox_var = tk.BooleanVar(glim_offline_menu, False, "slice_checkbox")
    slice_checkbox = tk.Checkbutton(slice_option_frame, text="Slice", variable=slice_checkbox_var, command=slice_checkbox_callback)
    slice_checkbox.grid(row=0, column=0, padx=5, pady=5)
    slice_pieces_entry_var = tk.IntVar(glim_offline_menu, 1, "slice_pieces")
    slice_pieces_entry = tk.Entry(slice_option_frame, font=normal_font, textvariable=slice_pieces_entry_var, state="disabled", width=5)
    slice_pieces_entry.grid(row=0, column=1, padx=5, pady=5)
    slice_unit_title = tk.Label(slice_option_frame, text="pieces", font=normal_font, state="disabled")
    slice_unit_title.grid(row=0, column=2, padx=5, pady=5)
    command_genrate_button = tk.Button(slice_option_frame, text="Generate", command=generate_command, state="disabled")
    command_genrate_button.grid(row=0, column=3, padx=5, pady=5)
    
    command_scrolledtext = scrolledtext.ScrolledText(glim_offline_menu, wrap=tk.WORD, width=45, height=10)
    command_scrolledtext.grid(row=2, column=0, columnspan=3, padx=5, pady=5)
    command_scrolledtext.tag_configure("highlight", background="darkblue", foreground="white")
    command_scrolledtext.config(cursor="arrow")
    command_scrolledtext.bind("<Button-1>", on_line_click)

    launch_button_frame = tk.Frame(glim_offline_menu)
    launch_button_frame.grid(row=3, column=0, columnspan=3, padx=5, pady=5)
    launch_button_var = tk.StringVar(glim_offline_menu, "Launch Full", "launch_button")
    launch_button = tk.Button(launch_button_frame, textvariable=launch_button_var, command=launch_glim_offline)
    launch_button.grid(row=0, column=0, padx=5, pady=5)
    launch_viewer_button = tk.Button(launch_button_frame, text="Launch Viewer", command=launch_viewwer)
    launch_viewer_button.grid(row=0, column=1, padx=5, pady=5)

    
    adjust_window_size(glim_offline_menu)
    # set_topmost(glim_offline_menu)
    glim_offline_menu.mainloop()

def hdl_waypoint():
    def select_pcd_file():
        file_path = filedialog.askopenfile(parent=hdl_waypoint_menu, initialdir=DataPath().pcd_map_dir_path)
        if file_path and ".pcd" in file_path.name:
            pcd_file_path_var.set(file_path.name)
            reset_pcd_file_path()
        else:
            # print("Wrong file path")
            reset_pcd_file_path()
            return
        
    def view_pcd_file():
        pcd_file_path = pcd_file_path_var.get()
        if not ".pcd" in pcd_file_path:
            return
        else:
            command = "pcl_viewer {}".format(pcd_file_path)
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
    
    def hdl_waypoint_launch():
        command = ""
        init_pos_x = 0.0
        init_pos_y = 0.0
        init_pos_z = 0.0
        init_ori_x = 0.0
        init_ori_y = 0.0
        init_ori_z = 0.0
        init_ori_w = 1.0
        
        if os.path.exists(os.path.join(bag_file_path_var.get(), "metadata.yaml")):
            with open(os.path.join(bag_file_path_var.get(), "metadata.yaml"), "r") as f:
                bag_duration = yaml.load(f, Loader=yaml.FullLoader)["rosbag2_bagfile_information"]["duration"]["nanoseconds"] * 1e-9
                
        for root, dirs, files in os.walk(DataPath().waypoint_dir_path):
            for file in files:
                if "converted" in file and ".yaml" in file:
                    with open(os.path.join(root, file), "r") as f:
                        documents = yaml.safe_load_all(f)
                        for document in documents:
                            for key in document:
                                # print(type(key))
                                # print(document)
                                if key == 0:
                                    init_pos_x = document[key]["position_x"]
                                    init_pos_y = document[key]["position_y"]
                                    init_pos_z = document[key]["position_z"]
                                    init_ori_x = document[key]["quaternion_x"]
                                    init_ori_y = document[key]["quaternion_y"]
                                    init_ori_z = document[key]["quaternion_z"]
                                    init_ori_w = document[key]["quaternion_w"]
                                    # print(init_ori_w, key, document)
                                    continue
                    command = "gnome-terminal --tab --name=HdlWaypoint --title=HdlWaypoint -- ros2 launch whill_navi2 hdl_localization_waypoint.launch.py {}".format(" ".join([
                        "globalmap_pcd:={}".format(pcd_file_path_var.get()),
                        "bag_file_path:={}".format(bag_file_path_var.get()),
                        "bag_play_duration:={}".format(bag_duration),
                        "init_pos_x:={}".format(init_pos_x),
                        "init_pos_y:={}".format(init_pos_y),
                        "init_pos_z:={}".format(init_pos_z),
                        "init_ori_x:={}".format(init_ori_x),
                        "init_ori_y:={}".format(init_ori_y),
                        "init_ori_z:={}".format(init_ori_z),
                        "init_ori_w:={}".format(init_ori_w)
                    ]))
                    break
                    # print(command)
                else:
                    command = "gnome-terminal --tab --name=HdlWaypoint --title=HdlWaypoint -- ros2 launch whill_navi2 hdl_localization_waypoint.launch.py {}".format(" ".join([
                        "globalmap_pcd:={}".format(pcd_file_path_var.get()),
                        "bag_file_path:={}".format(bag_file_path_var.get()),
                        "bag_play_duration:={}".format(bag_duration),
                    ]))
                    # print(command)
        
        if command == "":
            command = "gnome-terminal --tab --name=HdlWaypoint --title=HdlWaypoint -- ros2 launch whill_navi2 hdl_localization_waypoint.launch.py {}".format(" ".join([
                "globalmap_pcd:={}".format(pcd_file_path_var.get()),
                "bag_file_path:={}".format(bag_file_path_var.get()),
                "bag_play_duration:={}".format(bag_duration)
            ]))
        
        # print(command)
        subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
        # hdl_waypoint_log_box.insert(tk.END, command + "\n")
        # hdl_waypoint_log_box.see(tk.END)

    def waypoint_plot():
        
        def odometry_plot():
            command = "gnome-terminal --tab --name=OdometryPlot --title=OdometryPlot -- ros2 run waypoint_pkg odometry_plot --ros-args --remap __ns:=/comparision --param mode:=comparision --param waypoint_file_paths:='{}'".format(waypoint_file_path_list)
            # print(command)
            subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
            # hdl_waypoint_log_box.insert(tk.END, command + "\n")
            # hdl_waypoint_log_box.see(tk.END)
                
        def show_covariance():
            command = "gnome-terminal --tab --name=OdometryPlot --title=OdometryPlot -- ros2 run waypoint_pkg odometry_plot --ros-args --remap __ns:=/covariance --param mode:=covariance --param waypoint_file_paths:='{}'".format(waypoint_file_path_list)
            # print(command)
            subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
            # hdl_waypoint_log_box.insert(tk.END, command + "\n")
            # hdl_waypoint_log_box.see(tk.END)

        def select_waypoint_files():
            waypoint_file_paths = filedialog.askopenfilenames(parent=popup, initialdir=os.path.join(os.environ["HOME"], "Documents", "full_data"))
            for waypoint_file_path in waypoint_file_paths:
                if waypoint_file_path and ".yaml" in waypoint_file_path:
                    waypoint_file_path_list.append(waypoint_file_path)
                    
            waypoint_file_path_scrolltext.delete("1.0", tk.END)
            index = 0
            if len(waypoint_file_path_list) == 0:
                waypoint_file_path_scrolltext.insert(tk.END, "No file path yet")
            else:
                for waypoint_file_path in waypoint_file_path_list:
                    waypoint_file_path_scrolltext.insert(tk.END, "[ {} ] ".format(str(index)) + waypoint_file_path + "\n")
                    index += 1
        
        def reset_path_button():
            waypoint_file_path_list.clear()
            waypoint_file_path_scrolltext.delete("1.0", tk.END)
            waypoint_file_path_scrolltext.insert(tk.END, "No file path yet")
            waypoint_file_path_scrolltext.see(tk.END)
        
        waypoint_file_path_list = []
        
        popup = tk.Toplevel(hdl_waypoint_menu)
        popup.title("Waypoint Plot")
        popup.geometry("400x360")
        popup.grab_set()
        
        waypoint_file_path_title = tk.Label(popup, text="Waypoint file paths:", font=normal_font)
        waypoint_file_path_title.grid(row=1, column=0, sticky="w")
        waypoint_file_path_scrolltext = scrolledtext.ScrolledText(popup, wrap=tk.WORD, width=45, height=10)
        waypoint_file_path_scrolltext.grid(row=2, column=0, pady=3, sticky="w")
        waypoint_file_path_scrolltext.insert(tk.END, "No file path yet!")
        waypoint_file_path_scrolltext.see(tk.END)
        
        button_frame1 = tk.Frame(popup)
        button_frame1.grid(row=0, column=0, sticky="w")
        select_waypoint_files_button = tk.Button(button_frame1, text="Select", command=select_waypoint_files)
        select_waypoint_files_button.grid(row=0, column=0, sticky="w")
        reset_waypoint_path_button = tk.Button(button_frame1, text="Reset", command=reset_path_button)
        reset_waypoint_path_button.grid(row=0, column=1, sticky="w")
        button_frame2 = tk.Frame(popup)
        button_frame2.grid(row=3, column=0, sticky="w")
        compare_odometry_button = tk.Button(button_frame2, text="Comparision", command=odometry_plot)
        compare_odometry_button.grid(row=1, column=0, sticky="w")
        show_covariance_button = tk.Button(button_frame2, text="Covariance", command=show_covariance)
        show_covariance_button.grid(row=1, column=1, sticky="w")
        
        adjust_window_size(popup)
        popup.mainloop()

    def edit_waypoint():
        def select_waypoint_file():
            file_path = filedialog.askopenfile(parent=edit_waypoint_menu, initialdir=DataPath().waypoint_base_path)
            if file_path and ".yaml" in file_path.name:
                waypoint_file_path_var.set(file_path.name)
                reset_file_path()
            else:
                # print("Wrong file path")
                reset_file_path()
                return
        
        def select_pcd_file():
            file_path = filedialog.askopenfile(parent=edit_waypoint_menu, initialdir=DataPath().pcd_map_dir_path)
            if file_path and ".pcd" in file_path.name:
                pcd_file_path_var.set(file_path.name)
                edit_waypoint_log_box.insert(tk.END, "Selected pcd file path:\n")
                edit_waypoint_log_box.insert(tk.END, file_path.name)
                reset_file_path()
            else:
                # print("Wrong file path")
                reset_file_path()
                return
        
        def launch_edit_waypoint():
            command = "gnome-terminal --tab --name=WaypointEditor --title=WaypointEditor -- ros2 launch whill_navi2 glim_waypoint_editor.launch.py globalmap_pcd:={} waypoint_file_path:={} rewaypoint_file_path:={}".format(
                pcd_file_path_var.get(),
                waypoint_file_path_var.get(),
                os.path.join(DataPath().rewaypoint_dir_path, "re_" + os.path.basename(waypoint_file_path_var.get()))
            )
            subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
            make_dir_dict["rewaypoint_name"] = "re_" + os.path.basename(waypoint_file_path_var.get())
            # edit_waypoint_log_box.insert(tk.END, command + "\n")
            # edit_waypoint_log_box.see(tk.END)
            
            with open(make_dir_yaml_path, "w+") as f:
                write_data = {}
                write_data["make_dir_node"] = {}
                write_data["make_dir_node"]["ros__parameters"] = make_dir_dict
                yaml.safe_dump(write_data, f)
            
        def reset_file_path():
            edit_waypoint_log_box.delete("1.0", tk.END)
            edit_waypoint_log_box.insert(tk.END, "Waypoint file path : \n")
            edit_waypoint_log_box.insert(tk.END, waypoint_file_path_var.get() + "\n\n")
            edit_waypoint_log_box.insert(tk.END, "PCD file path : \n")
            edit_waypoint_log_box.insert(tk.END, pcd_file_path_var.get() + "\n\n")
            
        edit_waypoint_menu = tk.Toplevel(hdl_waypoint_menu)
        edit_waypoint_menu.title("EditWaypoint")
        edit_waypoint_menu.geometry("480x200")
        edit_waypoint_menu.grab_set()
        
        button_frame = tk.Frame(edit_waypoint_menu)
        button_frame.grid(row=0, column=0, sticky="w")
        select_waypoint_file_button = tk.Button(button_frame, text="Select Waypoint", command=select_waypoint_file)
        select_waypoint_file_button.grid(row=0, column=0, padx=5, sticky="w")
        launch_edit_waypoint_button = tk.Button(button_frame, text="Select PCD", command=select_pcd_file)
        launch_edit_waypoint_button.grid(row=0, column=1, padx=5, sticky="w")
        launch_edit_waypoint_button = tk.Button(button_frame, text="Edit Waypoint", command=launch_edit_waypoint)
        launch_edit_waypoint_button.grid(row=0, column=2, padx=5, sticky="w")
        
        # waypoint_file_path_title = tk.Label(edit_waypoint_menu, text="WaypointFilePath", font=bold_font)
        # waypoint_file_path_title.grid(row=1, column=0, sticky="w")
        
        waypoint_file_path_var = tk.StringVar(edit_waypoint_menu, "No waypoint file path yet")
        waypoint_file_path_frame = tk.Frame(edit_waypoint_menu)
        # waypoint_file_path_canvas = tk.Canvas(waypoint_file_path_frame, width=300, height=20)
        # waypoint_file_path_canvas.grid(row=0, column=0, sticky="w")
        # waypoint_file_path_scrollbar = tk.Scrollbar(waypoint_file_path_frame, orient="horizontal", command=waypoint_file_path_canvas.xview)
        # waypoint_file_path_scrollbar.grid(row=1, column=0)
        # waypoint_file_path_canvas.configure(xscrollcommand=waypoint_file_path_scrollbar.set)
        # waypoint_file_path_label_frame = tk.Frame(waypoint_file_path_canvas)
        # waypoint_file_path_label_frame.bind("<Configure>", lambda e: waypoint_file_path_canvas.configure(scrollregion=waypoint_file_path_canvas.bbox("all")))
        # waypoint_file_path_label = tk.Label(waypoint_file_path_label_frame, textvariable=waypoint_file_path_var, font=normal_font)
        # waypoint_file_path_label.grid(row=0, column=0)
        # waypoint_file_path_canvas.create_window((0, 0), window=waypoint_file_path_label_frame, anchor="nw")
        # waypoint_file_path_frame.grid_rowconfigure(0, weight=1)
        # waypoint_file_path_frame.grid_columnconfigure(0, weight=1)
        waypoint_file_path_frame.grid(row=1, column=0, pady=5, sticky="w")
        
        edit_waypoint_log_box = scrolledtext.ScrolledText(waypoint_file_path_frame, wrap=tk.WORD, width=50, height=7)
        edit_waypoint_log_box.grid(row=3, column=0, sticky="w")
        reset_file_path()
        
        adjust_window_size(edit_waypoint_menu)
        edit_waypoint_menu.mainloop()

    def convert_waypoint():
        def select_target_waypoint():
            file_path = filedialog.askopenfile(parent=convert_waypoint_menu, initialdir=DataPath().waypoint_dir_path)
            if file_path and ".yaml" in file_path.name:
                waypoint_path_dict["target_waypoint_path"] = file_path.name
                reset_convert_scrolledtext()
            
        def select_source_waypoint():
            file_path = filedialog.askopenfile(parent=convert_waypoint_menu, initialdir=DataPath().waypoint_dir_path)
            if file_path and ".yaml" in file_path.name:
                waypoint_path_dict["source_waypoint_path"] = file_path.name
                reset_convert_scrolledtext()
            
        def start_convert_waypoint():
            if not os.path.exists(pcd_file_path_var.get()):
                pcd_file_path = "no_path"
            else:
                pcd_file_path = pcd_file_path_var.get()
            command = "gnome-terminal --tab --name=ConvertWaypoint --title=ConvertWaypoint -- ros2 run whill_navi2 convert_waypoint_node --ros-args {}".format(" --param ".join([
                "--param target_waypoint_path:={}".format(waypoint_path_dict["target_waypoint_path"]),
                "source_waypoint_path:={}".format(waypoint_path_dict["source_waypoint_path"]),
                "input_pcd_path:={}".format(pcd_file_path)
            ]))
            # print(command)
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
            # convert_waypoint_scrolledtext.insert(tk.END, command + "\n")
            # process.wait(3.0)
        
        def reset_convert_scrolledtext():
            convert_waypoint_scrolledtext.delete("1.0", tk.END)
            convert_waypoint_scrolledtext.see("1.0")
            # convert_waypoint_scrolledtext.mark_set(tk.INSERT, "1.0")
            convert_waypoint_scrolledtext.insert(tk.END, " --- [ Target Waypoint Path ] --- " + "\n")
            convert_waypoint_scrolledtext.insert(tk.END, waypoint_path_dict["target_waypoint_path"] + "\n")
            convert_waypoint_scrolledtext.insert(tk.END, " --- [ Source Waypoint Path ] --- " + "\n")
            convert_waypoint_scrolledtext.insert(tk.END, waypoint_path_dict["source_waypoint_path"] + "\n")
            convert_waypoint_scrolledtext.see(tk.END)

        convert_waypoint_menu = tk.Toplevel(hdl_waypoint_menu)
        convert_waypoint_menu.grab_set()
        convert_waypoint_menu.title("Convert Waypoint")
        convert_waypoint_menu.geometry("540x150")
        
        waypoint_path_dict = {}
        waypoint_path_dict["target_waypoint_path"] = ""
        waypoint_path_dict["source_waypoint_path"] = ""
        
        convert_waypoint_button_frame = tk.Frame(convert_waypoint_menu)
        convert_waypoint_button_frame.grid(row=0, column=0, sticky="nw")
        select_target_waypoint_button = tk.Button(convert_waypoint_button_frame, text="Select Target Waypoint", command=select_target_waypoint)
        select_target_waypoint_button.grid(row=0, column=0, sticky="w")
        select_source_waypoint_button = tk.Button(convert_waypoint_button_frame, text="Select Source Waypoint", command=select_source_waypoint)
        select_source_waypoint_button.grid(row=1, column=0, sticky="w")
        start_convert_waypoint_button = tk.Button(convert_waypoint_button_frame, text="Start", command=start_convert_waypoint)
        start_convert_waypoint_button.grid(row=2, column=0, sticky="w")
        
        convert_waypoint_scrolledtext = scrolledtext.ScrolledText(convert_waypoint_menu, wrap=tk.WORD, width=50, height=5)
        convert_waypoint_scrolledtext.grid(row=0, column=1, sticky="w")
        reset_convert_scrolledtext()
        
        adjust_window_size(convert_waypoint_menu)
        convert_waypoint_menu.mainloop()
        
    def slice_pcd():
        command = "flatpak run org.cloudcompare.CloudCompare {}".format(pcd_file_path_var.get())
        subprocess.Popen(command, stdout=subprocess.PIPE, text=True, shell=True)
        
        # source_waypoint_log_box.insert(tk.END, command + "\n")
        # source_waypoint_log_box.see(tk.END)
    
    def reset_pcd_file_path():
        hdl_waypoint_log_box.delete("1.0", tk.END)
        hdl_waypoint_log_box.insert(tk.END, "PCD file path :" + "\n")
        hdl_waypoint_log_box.insert(tk.END, pcd_file_path_var.get() + "\n")

    hdl_waypoint_menu = tk.Toplevel(root)
    hdl_waypoint_menu.title("HdlWaypoint")
    hdl_waypoint_menu.geometry("560x200")
    hdl_waypoint_menu.grab_set()
    
    waypoint_frame = tk.Frame(hdl_waypoint_menu)
    waypoint_frame.grid(row=0, padx=5, column=1, sticky="n")
    waypoint_title = tk.Label(waypoint_frame, text="------  Waypoint  ------", font=bold_font)
    waypoint_title.grid(row=0, column=0, columnspan=3, sticky="w")
    hdl_waypoint_start_button = tk.Button(waypoint_frame, text="Create", command=hdl_waypoint_launch)
    hdl_waypoint_start_button.grid(row=1, column=0, sticky="w")
    waypoint_plot_button = tk.Button(waypoint_frame, text="Plot", command=waypoint_plot)
    waypoint_plot_button.grid(row=1, column=1, sticky="w")
    edit_waypoint_button = tk.Button(waypoint_frame, text="Edit", command=edit_waypoint)
    edit_waypoint_button.grid(row=1, column=2, sticky="w")

    pcd_frame = tk.Frame(hdl_waypoint_menu)
    pcd_frame.grid(row=0, column=0)
    # pcd_frame.grid_rowconfigure(0, weight=1)
    # pcd_frame.grid_columnconfigure(0, weight=1)
    pcd_file_path_title = tk.Label(pcd_frame, text="-----------------  PCD  -----------------", font=bold_font)
    pcd_file_path_title.grid(row=0, column=0, sticky="w")
    pcd_file_path_var = tk.StringVar(hdl_waypoint_menu, "No pcd file path yet")
    # pcd_file_path_canvas = tk.Canvas(pcd_frame, width=300, height=20)
    # pcd_file_path_canvas.grid(row=2, column=0, sticky="w")
    # pcd_file_path_scrollbar = tk.Scrollbar(pcd_frame, orient="horizontal", command=pcd_file_path_canvas.xview)
    # pcd_file_path_scrollbar.grid(row=3, column=0)
    # pcd_file_path_canvas.configure(xscrollcommand=pcd_file_path_scrollbar.set)
    # pcd_file_path_label_frame = tk.Frame(pcd_file_path_canvas)
    # pcd_file_path_label_frame.bind("<Configure>", lambda e: pcd_file_path_canvas.configure(scrollregion=pcd_file_path_canvas.bbox("all")))
    # pcd_file_path_label = tk.Label(pcd_file_path_label_frame, textvariable=pcd_file_path_var, font=normal_font)
    # pcd_file_path_label.grid(row=0, column=0)
    # pcd_file_path_canvas.create_window((0, 0), window=pcd_file_path_label_frame, anchor="nw")
    pcd_button_frame = tk.Frame(pcd_frame)
    pcd_button_frame.grid(row=1, column=0)
    select_pcd_file_button = tk.Button(pcd_button_frame, text="Select File", command=select_pcd_file)
    select_pcd_file_button.grid(row=0, column=0, sticky="w")
    view_pcd_file_button = tk.Button(pcd_button_frame, text="View PCD", command=view_pcd_file)
    view_pcd_file_button.grid(row=0, column=1, sticky="w")
    convert_waypoint_button = tk.Button(pcd_button_frame, text="Convert Waypoint", command=convert_waypoint)
    convert_waypoint_button.grid(row=0, column=2, sticky="w")
    slice_pcd_button = tk.Button(pcd_button_frame, text="Slice PCD", command=slice_pcd)
    slice_pcd_button.grid(row=1, column=0, sticky="w")

    hdl_waypoint_log_box = scrolledtext.ScrolledText(hdl_waypoint_menu, wrap=tk.WORD, width=50, height=5)
    hdl_waypoint_log_box.grid(row=2, column=0, columnspan=2, padx=10, pady=10, sticky="w")
    reset_pcd_file_path()
    
    adjust_window_size(hdl_waypoint_menu)
    hdl_waypoint_menu.mainloop()

def navigation():
    def on_whill_speed_changed(*args):
        if int(front_speed_var.get() * 3600 / 100) > 60:
            front_speed_var.set(float(60 * 100 / 3600)) 
        if int(front_speed_var.get() * 3600 / 100) < 8:
            front_speed_var.set(float(8 * 100 / 3600)) 
        if int(turn_speed_var.get() * 3600 / 100) > 35:
            turn_speed_var.set(float(35 * 100 / 3600)) 
        if int(turn_speed_var.get() * 3600 / 100) < 8:
            turn_speed_var.set(float(8 * 100 / 3600))
        whill_joy_dict["speed_profile_initial"][0] = int(front_speed_var.get() * 3600 / 100)
        whill_joy_dict["speed_profile_initial"][6] = int(turn_speed_var.get() * 3600 / 100)
        
        with open(get_file_path("whill_navi2", "nav2_params.yaml"), "r") as f:
            nav2_params_dict = yaml.safe_load(f)
        
        nav2_params_dict["velocity_smoother"]["ros__parameters"]["max_velocity"] = [front_speed_var.get() * 0.8, 0.0, turn_speed_var.get() * 0.8]
        nav2_params_dict["velocity_smoother"]["ros__parameters"]["min_velocity"] = [
            - whill_joy_dict["speed_profile_initial"][3] * 100 / 3600 * 0.8,
            0.0,
            - turn_speed_var.get() * 0.8
        ]
        # Reference: WHILL_Control_System_Protocol_Specifition.pdf - Page 8 - 3.2.7. SetVelocity command
        nav2_params_dict["velocity_smoother"]["ros__parameters"]["max_accel"] = [1.7 * 0.64, 0.0, 1.7 * 0.64]
        nav2_params_dict["velocity_smoother"]["ros__parameters"]["max_decel"] = [
            - float(whill_joy_dict["speed_profile_initial"][2] / 100.0) * 0.64, 
            0.0, 
            - float(whill_joy_dict["speed_profile_initial"][8] / 100.0) * 0.64
        ]
        
        with open(get_file_path("whill_navi2", "nav2_params.yaml"), "w+") as f:
            yaml.safe_dump(nav2_params_dict, f)
        
        with open(get_file_path("whill_navi2", "nav2_no_map_params.yaml"), "r") as f:
            nav2_params_dict = yaml.safe_load(f)
        
        nav2_params_dict["velocity_smoother"]["ros__parameters"]["max_velocity"] = [front_speed_var.get() * 0.8, 0.0, turn_speed_var.get() * 0.8]
        nav2_params_dict["velocity_smoother"]["ros__parameters"]["min_velocity"] = [
            - whill_joy_dict["speed_profile_initial"][3] * 100 / 3600 * 0.8,
            0.0,
            - turn_speed_var.get() * 0.8
        ]
        # Reference: WHILL_Control_System_Protocol_Specifition.pdf - Page 8 - 3.2.7. SetVelocity command
        nav2_params_dict["velocity_smoother"]["ros__parameters"]["max_accel"] = [1.7 * 0.64, 0.0, 1.7 * 0.64]
        nav2_params_dict["velocity_smoother"]["ros__parameters"]["max_decel"] = [
            - float(whill_joy_dict["speed_profile_initial"][2] / 100.0) * 0.64, 
            0.0, 
            - float(whill_joy_dict["speed_profile_initial"][8] / 100.0) * 0.64
        ]
        
        with open(get_file_path("whill_navi2", "nav2_no_map_params.yaml"), "w+") as f:
            yaml.safe_dump(nav2_params_dict, f)
                
        write_data = {}
        with open(ros2_whill_yaml_path, "r+") as f:
            write_data = yaml.safe_load(f)
        
        with open(ros2_whill_yaml_path, "w+") as f:
            write_data["whill_joy"]["ros__parameters"] = whill_joy_dict
            yaml.safe_dump(write_data, f)

    def launch_whill():
        launch_whill_command = "gnome-terminal --tab --name=Whill --title=Whill -- ros2 launch whill_navi2 kuaro_whill.launch.py publish_tf:=True"
        pid = subprocess.Popen(launch_whill_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True).pid
        process_pids.append(pid)
        # navigation_scrolledtext.insert(tk.END, launch_whill_command + "\n")
        # navigation_scrolledtext.see(tk.END)
                
    def launch_sensor():
        # def check_sensor_state(command : str) -> bool: 
        #     def read_output():
        #         try:
        #             while True:
        #                 stdout = process.stdout.readline()
        #                 if "WARNING" in stdout:
        #                     process.kill()
        #                     return
        #                 q.put(stdout)
        #                 if stdout:
        #                     navigation_scrolledtext.insert(tk.END, stdout + "\n")
        #                     navigation_scrolledtext.see(tk.END)
        #         except:
        #             process.kill()
        #             return

        #     process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
        #     q = queue.Queue(maxsize=10)
        #     pid_list.append(process.pid)
        #     try:
        #         read_thread = threading.Thread(target=read_output, daemon=True)
        #         read_thread.start()
        #         if q.empty():
        #             process.kill()
        #             return False
        #         result = q.get(timeout=1.0)
        #         process.kill()
        #         read_thread.join()
        #         if len(result) > 1:
        #             process.kill()
        #             return True
        #         process.kill()
        #         return False
        #     except:
        #         process.kill()
        #         return False

        # def check_sensor_state():
        #     def read_topic(pipe, sensor : str, output_queue : queue.Queue):
        #         for index in range(50):
        #             if output_queue.full():
        #                 pipe.close()
        #                 return
        #             time.sleep(0.1)
        #             for line in iter(pipe.readline, ''):
        #                 # print(line)
        #                 if sensor in line:
        #                     output_queue.put(line.strip())
        #         pipe.close()
                    
        #     velodyne_command = "ros2 topic echo /velodyne_points sensor_msgs/msg/PointCloud2"
        #     zed_camera_command = "ros2 topic echo /zed2i/zed_node/imu/data sensor_msgs/msg/Imu"
        #     ublox_command = "ros2 topic echo /ublox/nmea nmea_msgs/msg/Sentence"
        #     adis_imu_command = "ros2 topic echo /adis/imu/data sensor_msgs/msg/Imu"
        #     ylm20_command = "ros2 topic echo /lumotive_ros/pointcloud sensor_msgs/msg/PointCloud2"
        #     whill_command = "ros2 topic echo /whill/odometry nav_msgs/msg/Odometry"
            
        #     velodyne_ok = False
        #     zed_camera_ok = False
        #     ublox_ok = False
        #     adis_imu_ok = False
        #     ylm20_ok = False
        #     whill_ok = False
            
        #     read_velodyne_topic_queue = queue.Queue(3)
        #     read_zed_camera_topic_queue = queue.Queue(3)
        #     read_ublox_topic_queue = queue.Queue(3)
        #     read_adis_imu_topic_queue = queue.Queue(3)
        #     read_ylm20_topic_queue = queue.Queue(3)
        #     read_whill_topic_queue = queue.Queue(3)
            
        #     read_velodyne_topic = subprocess.Popen(velodyne_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True) if velodyne_state_var.get() else None
        #     read_zed_camera_topic = subprocess.Popen(zed_camera_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True) if zed_camera_state_var.get() else None
        #     read_ublox_topic = subprocess.Popen(ublox_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True) if ublox_state_var.get() else None
        #     read_adis_imu_topic = subprocess.Popen(adis_imu_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True) if adis_imu_state_var.get() else None
        #     read_ylm20_topic = subprocess.Popen(ylm20_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True) if ylm20_state_var.get() else None
        #     read_whill_topic = subprocess.Popen(whill_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
            
        #     # print(read_velodyne_topic, read_zed_camera_topic, read_ublox_topic, read_adis_imu_topic, read_ylm20_topic, read_whill_topic)
        #     if read_velodyne_topic is not None:
        #         read_velodyne_topic_thread = threading.Thread(target=read_topic, daemon=True, args=(read_velodyne_topic.stdout, "velodyne", read_velodyne_topic_queue))
        #         read_velodyne_topic_thread.start()
        #         read_velodyne_topic_thread.join(10.0)
        #     if read_zed_camera_topic is not None:
        #         read_zed_camera_topic_thread = threading.Thread(target=read_topic, daemon=True, args=(read_zed_camera_topic.stdout, "zed2i_imu_link", read_zed_camera_topic_queue))
        #         read_zed_camera_topic_thread.start()
        #         read_zed_camera_topic_thread.join(10.0)
        #     if read_ublox_topic is not None:
        #         read_ublox_topic_thread = threading.Thread(target=read_topic, daemon=True, args=(read_ublox_topic.stdout, "ublox", read_ublox_topic_queue))
        #         read_ublox_topic_thread.start()
        #         read_ublox_topic_thread.join(10.0)
        #     if read_adis_imu_topic is not None:
        #         read_adis_imu_topic_thread = threading.Thread(target=read_topic, daemon=True, args=(read_adis_imu_topic.stdout, "imu_adis", read_adis_imu_topic_queue))
        #         read_adis_imu_topic_thread.start()
        #         read_adis_imu_topic_thread.join(10.0)
        #     if read_ylm20_topic is not None:
        #         read_ylm20_topic_thread = threading.Thread(target=read_topic, daemon=True, args=(read_ylm20_topic.stdout, "m20_lidar", read_ylm20_topic_queue))
        #         read_ylm20_topic_thread.start()
        #         read_ylm20_topic_thread.join(10.0)
        #     if read_whill_topic is not None:
        #         read_whill_topic_thread = threading.Thread(target=read_topic, daemon=True, args=(read_whill_topic.stdout, "base_link", read_whill_topic_queue))
        #         read_whill_topic_thread.start()
        #         read_whill_topic_thread.join(10.0)

        #     for index in range(50):                
        #         time.sleep(0.1)
        #         if not read_velodyne_topic_queue.empty() and not velodyne_ok:
        #             velodyne_state_checkbox.config(fg="green")
        #             velodyne_ok = True
        #         if not read_zed_camera_topic_queue.empty() and not zed_camera_ok:
        #             zed_camera_state_checkbox.config(fg="green")
        #             zed_camera_ok = True
        #         if not read_ublox_topic_queue.empty() and not ublox_ok:
        #             ublox_state_checkbox.config(fg="green")
        #             ublox_ok = True
        #         if not read_adis_imu_topic_queue.empty() and not adis_imu_ok:
        #             adis_imu_state_checkbox.config(fg="green")
        #             adis_imu_ok = True
        #         if not read_ylm20_topic_queue.empty() and not ylm20_ok:
        #             ylm20_state_checkbox.config(fg="green")
        #             ylm20_ok = True
        #         if not read_whill_topic_queue.empty() and not whill_ok:
        #             whill_state_label.config(fg="green")
        #             whill_ok = True
            
        #     read_velodyne_topic_queue.task_done()
        #     read_zed_camera_topic_queue.task_done()
        #     read_ublox_topic_queue.task_done()
        #     read_adis_imu_topic_queue.task_done()
        #     read_ylm20_topic_queue.task_done()
        #     read_whill_topic_queue.task_done()
            
        #     read_velodyne_topic.kill()
        #     for index in range(3):
        #         os.kill(read_velodyne_topic.pid + index, signal.SIGTERM)
        #     read_velodyne_topic.wait(10.0)
        #     read_zed_camera_topic.kill()
        #     read_zed_camera_topic.wait(10.0)
        #     read_ublox_topic.kill()
        #     read_ublox_topic.wait(10.0)
        #     read_adis_imu_topic.kill()
        #     read_adis_imu_topic.wait(10.0)
        #     read_ylm20_topic.kill()
        #     read_ylm20_topic.wait(10.0)
        #     read_whill_topic.kill()
        #     for index in range(3):
        #         os.kill(read_whill_topic.pid + index, signal.SIGTERM)
        #     read_whill_topic.wait(10.0)

        def velodyne_callback(msg : PointCloud2):
            if msg.header.frame_id:
                velodyne_state_checkbox.config(fg="green")
        
        def zed_camera_callback(msg : Imu):
            if msg.header.frame_id:
                # print(msg.header.frame_id)
                zed_camera_state_checkbox.config(fg="green")
        
        def ublox_callback(msg : Sentence):
            if msg.header.frame_id:
                # print(msg.header.frame_id)
                ublox_state_checkbox.config(fg="green")
        
        def adis_imu_callback(msg : Imu):
            if msg.header.frame_id:
                # print(msg.header.frame_id)
                adis_imu_state_checkbox.config(fg="green")
        
        def ylm20_callback(msg : PointCloud2):
            if msg.header.frame_id:
                # print(msg.header.frame_id)
                ylm20_state_checkbox.config(fg="green")
        
        def whill_callback(msg : Odometry):
            if msg.header.frame_id:
                # print(msg.header.frame_id)
                whill_state_label.config(fg="green")
        
        # velodyne_state_label.config(fg="green")
        launch_sensor_command = "gnome-terminal --tab --name=Sensor --title=Sensor -- ros2 launch whill_navi2 sensor.launch.py {}".format(" ".join([
            "use_velodyne:={}".format(str(velodyne_state_var.get())),
            "use_zed_camera:={}".format(str(zed_camera_state_var.get())),
            "use_ublox:={}".format(str(ublox_state_var.get())),
            "use_adis_imu:={}".format(str(adis_imu_state_var.get())),
            "use_m20_lidar:={}".format(str(ylm20_state_var.get()))
        ]))
        pid = subprocess.Popen(launch_sensor_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True).pid
        process_pids.append(pid)

        # navigation_scrolledtext.insert(tk.END, launch_sensor_command + "\n")
        # navigation_scrolledtext.see(tk.END)
        
        rclpy.init()
        
        check_sensor_topic_node = Node("check_sensor_topic_node")
        if velodyne_state_var.get():
            velodyne_sub = check_sensor_topic_node.create_subscription(PointCloud2, "velodyne_points", velodyne_callback, 2)
        if zed_camera_state_var.get():
            zed_camera_sub = check_sensor_topic_node.create_subscription(Imu, "zed2i/zed_node/imu/data", zed_camera_callback, 2)
        if ublox_state_var.get():
            ublox_sub = check_sensor_topic_node.create_subscription(Sentence, "ublox/nmea", ublox_callback, 2)
        if adis_imu_state_var.get():
            adis_imu_sub = check_sensor_topic_node.create_subscription(Imu, "adis/imu/data", adis_imu_callback, 2)
        if ylm20_state_var.get():
            ylm20_sub = check_sensor_topic_node.create_subscription(PointCloud2, "lumotive_ros/pointcloud", ylm20_callback, 2)
        whill_sub = check_sensor_topic_node.create_subscription(Odometry, "whill/odometry", whill_callback, 2)
        
        for index in range(20):
            check_sensor_topic_node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0, nanoseconds=int(1e8)))
            rclpy.spin_once(node=check_sensor_topic_node)
        
        rclpy.shutdown()
        check_sensor_topic_node.destroy_node()

    def select_pcd_file():
        # pcd_file_path = filedialog.askopenfile(parent=navigation_menu, initialdir=DataPath().pcd_map_dir_path).name
        pcd_file_paths = filedialog.askopenfilenames(parent=navigation_menu, initialdir=os.path.join(os.environ["HOME"], "Documents", "full_data"), filetypes=[("PcdFile", "*.pcd")])
        for pcd_file_path in pcd_file_paths:
            if pcd_file_path and ".pcd" in pcd_file_path:
                pcd_file_path_list.append(pcd_file_path)
                
        pcd_file_path_scrolltext.delete("1.0", tk.END)
        index = 0
        if len(pcd_file_path_list) == 0:
            pcd_file_path_scrolltext.insert(tk.END, "No file path yet")
        else:
            for pcd_file_path in pcd_file_path_list:
                pcd_file_path_scrolltext.insert(tk.END, "[ {} ] ".format(str(index)) + pcd_file_path + "\n")
                index += 1
            
    def view_pcd():
        for pcd_file_path in pcd_file_path_list:
            command = "pcl_viewer {}".format(pcd_file_path)
            subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
            # navigation_scrolledtext.insert(tk.END, command + "\n")
            # navigation_scrolledtext.see(tk.END)

    def launch_localizer():
        if has_map_var.get():
            if use_remap_var.get():
                launch_localizer_command = "gnome-terminal --tab --name=Localizer --title=Localizer -- ros2 launch whill_navi2 hdl_localization.launch.py globalmap_pcd:={} gridmap_topic:={}".format(pcd_file_path_list[0], "hdl_global_localization/map")
                pid = subprocess.Popen(launch_localizer_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True).pid
                process_pids.append(pid)
            elif use_hdl_bbs_map_var.get():
                launch_localizer_command = "gnome-terminal --tab --name=Localizer --title=Localizer -- ros2 launch whill_navi2 hdl_localization.launch.py globalmap_pcd:={} gridmap_topic:={}".format(pcd_file_path_list[0], "map")
                pid = subprocess.Popen(launch_localizer_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True).pid
                process_pids.append(pid)
        elif no_map_var.get():
            launch_localizer_command = "gnome-terminal --tab --name=Localizer --title=Localizer -- ros2 launch whill_navi2 lio_gnss_localization.launch.py {}".format(
                " ".join([
                    "T_utm_topomap_file_path:={}".format(T_utm_topomap_file_path_var.get()),
                    "T_topomap_liomap_file_path:={}".format(T_topomap_liomap_file_path_var.get()),
                    "front_speed:={}".format(front_speed_var.get()),
                    "turn_speed:={}".format(turn_speed_var.get())
                ])
            )
            pid = subprocess.Popen(launch_localizer_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True).pid
            process_pids.append(pid)
            navigation_scrolledtext.insert(tk.END, launch_localizer_command + "\n")
            navigation_scrolledtext.see(tk.END)

    def select_gridmap():
        # grid_map_path = filedialog.askopenfile(parent=navigation_menu, initialdir=DataPath().remap_dir_path).name
        grid_map_paths = filedialog.askopenfilenames(parent=navigation_menu, initialdir=os.path.join(os.environ["HOME"], "Documents", "full_data"), filetypes=[("YamlFile", "*.yaml")])
        for grid_map_path in grid_map_paths:
            if grid_map_path and ".yaml" in grid_map_path:
                gridmap_file_path_list.append(grid_map_path)
                
        gridmap_file_path_scrolltext.delete("1.0", tk.END)
        index = 0
        if len(gridmap_file_path_list) == 0:
            gridmap_file_path_scrolltext.insert(tk.END, "No file path yet")
        else:
            for grid_map_path in gridmap_file_path_list:
                gridmap_file_path_scrolltext.insert(tk.END, "[ {} ] ".format(str(index)) + grid_map_path + "\n")
                index += 1
    
    def view_gridmap():
        for gridmap_file_path in gridmap_file_path_list:
            image_file_path = os.path.splitext(gridmap_file_path)[0] + ".pgm"
            command = "eog {}".format(image_file_path)
            subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
            # navigation_scrolledtext.insert(tk.END, command + "\n")
            # navigation_scrolledtext.see(tk.END)

    def launch_navigator():
        if has_map_var.get():
            if use_remap_var.get():
                launch_navigator_command = "gnome-terminal --tab --name=Navigator --title=Navigator -- ros2 launch whill_navi2 navigation.launch.py params_file:={} map_yaml_filename:={} use_map_server:={}".format(
                    get_file_path("whill_navi2", "nav2_params.yaml"),
                    gridmap_file_path_list[0],
                    # "no_path",
                    "True"
                    # "False"
                )
                pid = subprocess.Popen(launch_navigator_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True).pid
                process_pids.append(pid)
            elif use_hdl_bbs_map_var.get():
                launch_navigator_command = "gnome-terminal --tab --name=Navigator --title=Navigator -- ros2 launch whill_navi2 navigation.launch.py params_file:={} map_yaml_filename:={} use_map_server:={}".format(
                    get_file_path("whill_navi2", "nav2_params.yaml"),
                    # gridmap_file_path_list[0],
                    "no_path",
                    # "True"
                    "False"
                )
                pid = subprocess.Popen(launch_navigator_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True).pid
                process_pids.append(pid)
                
        elif no_map_var.get():
            launch_navigator_command = "gnome-terminal --tab --name=Navigator --title=Navigator -- ros2 launch whill_navi2 navigation.launch.py {}".format(
                " ".join([
                    "params_file:={}".format(get_file_path("whill_navi2", "nav2_no_map_params.yaml")),
                    "use_map_server:={}".format("False")
                ])
            )
            pid = subprocess.Popen(launch_navigator_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True).pid
            process_pids.append(pid)

            navigation_scrolledtext.insert(tk.END, launch_navigator_command + "\n")
            navigation_scrolledtext.see(tk.END)
            
    def select_waypoint():
        waypoint_file_paths = filedialog.askopenfilenames(parent=navigation_menu, initialdir=os.path.join(os.environ["HOME"], "Documents", "full_data"))
        for waypoint_file_path in waypoint_file_paths:
            if waypoint_file_path and ".yaml" in waypoint_file_path:
                waypoint_file_path_list.append(waypoint_file_path)
                
        waypoint_file_path_scrolltext.delete("1.0", tk.END)
        index = 0
        if len(waypoint_file_path_list) == 0:
            waypoint_file_path_scrolltext.insert(tk.END, "No file path yet")
        else:
            for waypoint_file_path in waypoint_file_path_list:
                waypoint_file_path_scrolltext.insert(tk.END, "[ {} ] ".format(str(index)) + waypoint_file_path + "\n")
                index += 1
    
    def view_waypoint():
        command = "ros2 run waypoint_pkg odometry_plot --ros-args --param mode:=covariance --param waypoint_file_paths:='{}'".format(waypoint_file_path_list)
        # print(command)
        subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
        # navigation_scrolledtext.insert(tk.END, command + "\n")
        # navigation_scrolledtext.see(tk.END)
        
    def launch_whill_navi():
        if len(pcd_file_path_list) != 0:
            pcd_file_path0 = pcd_file_path_list.pop(0)
            gridmap_file_path0 = gridmap_file_path_list.pop(0)
        if no_map_var.get():
            use_map = False
            map_frame_id = "topomap"
        elif has_map_var.get():
            use_map = True
            map_frame_id = "map"
        
        if use_remap_var.get():
            launch_whill_navi_command = "gnome-terminal --tab --name=WhillNavi --title=WhillNavi -- ros2 run whill_navi2 whill_navi2_node --ros-args --param {}".format(" --param ".join([
                "waypoint_file_paths:='{}'".format(waypoint_file_path_list),
                "pcd_file_paths:='{}'".format(pcd_file_path_list),
                "use_remap:={}".format(True),
                "gridmap_file_paths:='{}'".format(gridmap_file_path_list),
                # "gridmap_file_paths:='{}'".format(["no", "any", "path"]),
                "is_loop:={}".format(is_loop_var.get()),
                "loop_limit:={}".format(loop_limit_var.get()),
                "use_map:={}".format(use_map),
                "map_frame_id:={}".format(map_frame_id)
            ]))
            pid = subprocess.Popen(launch_whill_navi_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True).pid
            process_pids.append(pid)
        elif use_hdl_bbs_map_var.get():
            launch_whill_navi_command = "gnome-terminal --tab --name=WhillNavi --title=WhillNavi -- ros2 run whill_navi2 whill_navi2_node --ros-args --param {}".format(" --param ".join([
                "waypoint_file_paths:='{}'".format(waypoint_file_path_list),
                "pcd_file_paths:='{}'".format(pcd_file_path_list),
                "use_remap:={}".format(False),
                # "gridmap_file_paths:='{}'".format(gridmap_file_path_list),
                "gridmap_file_paths:='{}'".format(["no", "any", "path"]),
                "is_loop:={}".format(is_loop_var.get()),
                "loop_limit:={}".format(loop_limit_var.get()),
                "use_map:={}".format(use_map),
                "map_frame_id:={}".format(map_frame_id)
            ]))
            pid = subprocess.Popen(launch_whill_navi_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True).pid
            process_pids.append(pid)
        else:
            launch_whill_navi_command = "gnome-terminal --tab --name=WhillNavi --title=WhillNavi -- ros2 run whill_navi2 whill_navi2_node --ros-args --param {}".format(" --param ".join([
                "waypoint_file_paths:='{}'".format(waypoint_file_path_list),
                # "pcd_file_paths:='{}'".format(["no", "any", "path"]),
                "use_remap:={}".format(False),
                # "gridmap_file_paths:='{}'".format(gridmap_file_path_list),
                # "gridmap_file_paths:='{}'".format(["no", "any", "path"]),
                "is_loop:={}".format(is_loop_var.get()),
                "loop_limit:={}".format(loop_limit_var.get()),
                "use_map:={}".format(use_map),
                "map_frame_id:={}".format(map_frame_id)
            ]))
            pid = subprocess.Popen(launch_whill_navi_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True).pid
            process_pids.append(pid)
        
        if len(pcd_file_path_list) != 0:
            pcd_file_path_list.insert(0, pcd_file_path0)
            gridmap_file_path_list.insert(0, gridmap_file_path0)
        
        launch_emergency_stop_command = "gnome-terminal --tab --name=EmergencyStop --title=EmergencyStop -- ros2 run whill_navi2 lidar_stop_node --ros-args --param stop_distance:={} --param scan_min_range:={} --param scan_max_range:={}".format(
            stop_distance_var.get(), scan_min_range_var.get(), scan_max_range_var.get()
        )
        if ylm20_state_var.get():
            pid = subprocess.Popen(launch_emergency_stop_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True).pid
            process_pids.append(pid)
            # navigation_scrolledtext.insert(tk.END, launch_emergency_stop_command + "\n")
            # navigation_scrolledtext.see(tk.END)

        navigation_scrolledtext.insert(tk.END, launch_whill_navi_command + "\n")
        navigation_scrolledtext.see(tk.END)

    def launch_rviz():
        # velodyne_state_label.config(fg="green")
        if has_map_var.get():
            command = "gnome-terminal --tab --name=Rviz2 --title=Rviz2 -- ros2 run rviz2 rviz2 -d {}".format(get_file_path("whill_navi2", "glim_navi.rviz"))
        elif no_map_var.get():
            command = "gnome-terminal --tab --name=Rviz2 --title=Rviz2 -- ros2 run rviz2 rviz2 -d {}".format(get_file_path("whill_navi2", "no_map_navi.rviz"))
        # print(launch_sensor_command)
        pid = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True).pid
        process_pids.append(pid)
        # navigation_scrolledtext.insert(tk.END, command + "\n")
        # navigation_scrolledtext.see(tk.END)

    def reset_states():
        velodyne_state_var.set(False)
        zed_camera_state_var.set(False)
        ublox_state_var.set(False)
        adis_imu_state_var.set(False)
        ylm20_state_var.set(False)
        # velodyne_state_label.config(fg="red")
        # zed_camera_state_label.config(fg="red")
        # ublox_state_label.config(fg="red")
        # adis_imu_state_label.config(fg="red")
        # ylm20_state_label.config(fg="red")
        whill_state_label.config(fg="red")
        on_sensor_changed()
    
    def reset_pcd_path():
        pcd_file_path_list.clear()
        pcd_file_path_scrolltext.delete("1.0", tk.END)
        pcd_file_path_scrolltext.insert(tk.END, "No file path yet")
    
    def reset_gridmap_path():
        gridmap_file_path_list.clear()
        gridmap_file_path_scrolltext.delete("1.0", tk.END)
        gridmap_file_path_scrolltext.insert(tk.END, "No file path yet")
    
    def reset_waypoint_path():
        waypoint_file_path_list.clear()
        waypoint_file_path_scrolltext.delete("1.0", tk.END)
        waypoint_file_path_scrolltext.insert(tk.END, "No file path yet")
    
    def reset_transform_path():
        pcd_file_path_scrolltext.delete("1.0", tk.END)
        pcd_file_path_scrolltext.insert(tk.END, "T_utm_topomap : " + "\n")
        pcd_file_path_scrolltext.insert(tk.END, T_utm_topomap_file_path_var.get() + "\n")
        pcd_file_path_scrolltext.insert(tk.END, "T_topomap_liomap :" + "\n")
        pcd_file_path_scrolltext.insert(tk.END, T_topomap_liomap_file_path_var.get() + "\n")
    
    def on_sensor_changed():
        if velodyne_state_var.get():
            velodyne_state_checkbox.config(fg="red")
        else:
            velodyne_state_checkbox.config(fg="gray")
        if zed_camera_state_var.get():
            zed_camera_state_checkbox.config(fg="red")
        else:
            zed_camera_state_checkbox.config(fg="gray")
        if adis_imu_state_var.get():
            adis_imu_state_checkbox.config(fg="red")
        else:
            adis_imu_state_checkbox.config(fg="gray")
        if ylm20_state_var.get():
            ylm20_state_checkbox.config(fg="red")
        else:
            ylm20_state_checkbox.config(fg="gray")
        if ublox_state_var.get():
            ublox_state_checkbox.config(fg="red")
        else:
            ublox_state_checkbox.config(fg="gray")
    
    def check_scan_range(*args):
        if scan_min_range_var.get() <= -60.0:
            scan_min_range_var.set(-60.0)
        if scan_max_range_var.get() >= 60.0:
            scan_max_range_var.set(60.0)
    
    def reset_localizer_navigator():
        localizer_label.config(fg="gray")
        navigator_label.config(fg="gray")
        select_pcd_file_button.config(state="disabled")
        select_T_utm_topomap_button.config(state="disabled")
        view_pcd_file_button.config(state="disabled")
        select_T_topomap_liomap_button.config(state="disabled")
        launch_localizer_button.config(state="disabled")
        reset_pcd_path_button.config(state="disabled")
        select_gridmap_button.config(state="disabled")
        view_gridmap_button.config(state="disabled")
        launch_navigator_button.config(state="disabled")
        reset_gridmap_path_button.config(state="disabled")
        use_remap_checkbox.config(state="disabled")
        use_hdl_bbs_map_checkbox.config(state="disabled")
        reset_pcd_path()
        reset_gridmap_path()
        reset_navigator()

    def activate_has_map():
        localizer_label.config(fg="black")
        navigator_label.config(fg="black")
        select_pcd_file_button.config(state="normal")
        view_pcd_file_button.config(state="normal")
        select_T_utm_topomap_button.grid_forget()
        select_T_topomap_liomap_button.grid_forget()
        select_pcd_file_button.grid(row=1, column=0, sticky="w")
        view_pcd_file_button.grid(row=1, column=1, sticky="w")
        launch_localizer_button.config(state="normal")
        reset_pcd_path_button.config(state="normal")
        select_gridmap_button.config(state="normal")
        view_gridmap_button.config(state="normal")
        launch_navigator_button.config(state="normal")
        reset_gridmap_path_button.config(state="normal")

    def activate_no_map():
        localizer_label.config(fg="black")
        navigator_label.config(fg="black")
        select_T_utm_topomap_button.config(state="normal")
        select_T_topomap_liomap_button.config(state="normal")
        select_pcd_file_button.grid_forget()
        view_pcd_file_button.grid_forget()
        select_T_utm_topomap_button.grid(row=1, column=0, sticky="w")
        select_T_topomap_liomap_button.grid(row=1, column=1, sticky="w")
        launch_localizer_button.config(state="normal")
        launch_navigator_button.config(state="normal")

    def on_localizer_changed():
        has_map_state = has_map_var.get()
        no_map_state = no_map_var.get()
        reset_navigator()
        if has_map_state and no_map_state:
            has_map_var.set(False)
            no_map_var.set(False)
            reset_localizer_navigator()
            reset_navigator()
        elif has_map_state:
            reset_localizer_navigator()
            activate_has_map()
            reset_navigator()
            use_remap_checkbox.config(state="normal")
            use_hdl_bbs_map_checkbox.config(state="normal")
        elif no_map_state:
            reset_localizer_navigator()
            activate_no_map()
            reset_navigator()
            launch_navigator_button.config(state="normal")
            reset_transform_path()
        else:
            reset_navigator()
            reset_localizer_navigator()

    def on_navigator_changed():
        use_remap_state = use_remap_var.get()
        use_hdl_bbs_map_state = use_hdl_bbs_map_var.get()
        if use_remap_state and use_hdl_bbs_map_state:
            use_remap_var.set(False)
            use_hdl_bbs_map_var.set(False)
            reset_navigator()
            return
        elif use_remap_state:
            reset_navigator()
            activate_remap_navigator()
            return
        elif use_hdl_bbs_map_state:
            reset_navigator()
            activate_hdl_bbs_navigator()
            reset_gridmap_path()
            return
        else:
            reset_navigator()
            return
    
    def reset_navigator():
        select_gridmap_button.config(state="disabled")
        view_gridmap_button.config(state="disabled")
        reset_gridmap_path_button.config(state="disabled")
        launch_navigator_button.config(state="disabled")
    
    def activate_remap_navigator():
        select_gridmap_button.config(state="normal")
        view_gridmap_button.config(state="normal")
        reset_gridmap_path_button.config(state="normal")
        launch_navigator_button.config(state="normal")
    
    def activate_hdl_bbs_navigator():
        select_gridmap_button.config(state="disabled")
        view_gridmap_button.config(state="disabled")
        reset_gridmap_path_button.config(state="disabled")
        launch_navigator_button.config(state="normal")

    def select_T_utm_topomap_file():
        # print("select_T_utm_topomap_file")
        T_utm_topomap_file_path = filedialog.askopenfilename(parent=navigation_menu, initialdir=DataPath().waypoint_base_path)
        if ".yaml" in T_utm_topomap_file_path:
            T_utm_topomap_file_path_var.set(T_utm_topomap_file_path)
        reset_transform_path()
    
    def select_T_topomap_liomap_file():
        print("select_T_topomap_liomap_file")
        T_topomap_liomap_file_path = filedialog.askopenfilename(parent=navigation_menu, initialdir=DataPath().waypoint_base_path)
        if ".yaml" in T_topomap_liomap_file_path:
            T_topomap_liomap_file_path_var.set(T_topomap_liomap_file_path)
        reset_transform_path()

    pcd_file_path_list = []
    gridmap_file_path_list = []
    waypoint_file_path_list = []
    process_pids = []
    navigation_menu = tk.Toplevel(root)
    navigation_menu.title("Navigation")
    navigation_menu.geometry("800x520")
    navigation_menu.grab_set()
    
    # canvas = tk.Canvas(navigation_menu)
    # scrollbar = tk.Scrollbar(navigation_menu, orient="vertical", command=canvas.yview)
    # scrollbar.pack(side="right", fill="y")
    # canvas.pack(side="left", fill="both", expand=True)
    # canvas.config(yscrollcommand=scrollbar.set)

    # main_frame = tk.Frame(canvas)
    # canvas.create_window((0, 0), window=main_frame, height=700, anchor="nw")
    # canvas.configure(scrollregion=canvas.bbox("all"))

    # Whill section
    whill_frame = tk.Frame(navigation_menu)
    whill_frame.grid(row=0, column=0, sticky="nw")
        
    whill_label = tk.Label(whill_frame, text="------------  Whill  ------------", font=bold_font)
    whill_label.grid(row=0, column=0, columnspan=4, pady=5, sticky="w")
    front_speed_label = tk.Label(whill_frame, text="Max front speed:", font=normal_font)
    front_speed_label.grid(row=1, column=0, pady=5, sticky="w")
    front_speed_var = tk.DoubleVar(navigation_menu, 0.8)
    front_speed_entry = tk.Entry(whill_frame, width=5, textvariable=front_speed_var, font=normal_font)
    front_speed_entry.grid(row=1, column=1, pady=5, sticky="w")
    front_speed_entry.bind("<FocusOut>", on_whill_speed_changed)
    front_speed_unit_label = tk.Label(whill_frame, text="m/s", font=normal_font)
    front_speed_unit_label.grid(row=1, column=1, padx=60, pady=5, sticky="w")
    turn_speed_label = tk.Label(whill_frame, text="Max turn speed:", font=normal_font)
    turn_speed_label.grid(row=2, column=0, pady=5, sticky="w")
    turn_speed_var = tk.DoubleVar(whill_frame, 0.8)
    turn_speed_entry = tk.Entry(whill_frame, width=5, textvariable=turn_speed_var, font=normal_font)
    turn_speed_entry.grid(row=2, column=1, pady=5, sticky="w")
    turn_speed_entry.bind("<FocusOut>", on_whill_speed_changed)
    turn_speed_unit_label = tk.Label(whill_frame, text="m/s", font=normal_font)
    turn_speed_unit_label.grid(row=2, column=1, padx=60, pady=5, sticky="w")
    launch_whill_button = tk.Button(whill_frame, text="Launch whill", command=launch_whill)
    launch_whill_button.grid(row=3, column=0, sticky="w")
    on_whill_speed_changed()
    
    # Sensor section
    sensor_frame = tk.Frame(navigation_menu)
    sensor_frame.grid(row=0, column=1, sticky="nw")
    
    sensor_label = tk.Label(sensor_frame, text="------------  Sensor  ------------", font=bold_font)
    sensor_label.grid(row=0, column=0, columnspan=4, pady=10, sticky="w")
    velodyne_state_var = tk.BooleanVar(navigation_menu, False)
    velodyne_state_checkbox = tk.Checkbutton(sensor_frame, text="Velodyne", variable=velodyne_state_var, font=normal_font, fg="gray", command=on_sensor_changed)
    velodyne_state_checkbox.grid(row=1, column=0, padx=5, sticky="w")
    zed_camera_state_var = tk.BooleanVar(navigation_menu, False)
    zed_camera_state_checkbox = tk.Checkbutton(sensor_frame, text="Zed2i Camera", variable=zed_camera_state_var, font=normal_font, fg="gray", command=on_sensor_changed)
    zed_camera_state_checkbox.grid(row=1, column=1, padx=5, sticky="w")
    adis_imu_state_var = tk.BooleanVar(navigation_menu, False)
    adis_imu_state_checkbox = tk.Checkbutton(sensor_frame, text="Adis16465", variable=adis_imu_state_var, font=normal_font, fg="gray", command=on_sensor_changed)
    adis_imu_state_checkbox.grid(row=2, column=0, padx=5, sticky="w")
    ublox_state_var = tk.BooleanVar(navigation_menu, False)
    ublox_state_checkbox = tk.Checkbutton(sensor_frame, text="Ublox", variable=ublox_state_var, font=normal_font, fg="gray", command=on_sensor_changed)
    ublox_state_checkbox.grid(row=2, column=1, padx=5, sticky="w")
    ylm20_state_var = tk.BooleanVar(navigation_menu, False)
    ylm20_state_checkbox = tk.Checkbutton(sensor_frame, text="Ylm20", variable=ylm20_state_var, font=normal_font, fg="gray", command=on_sensor_changed)
    ylm20_state_checkbox.grid(row=3, column=0, padx=5, sticky="w")
    
    # velodyne_state_label = tk.Label(sensor_frame, text="Velodyne", font=normal_font, fg="red")
    # velodyne_state_label.grid(row=1, column=0, padx=5, sticky="w")
    # zed_camera_state_label = tk.Label(sensor_frame, text="Zed2i Camera", font=normal_font, fg="red")
    # zed_camera_state_label.grid(row=1, column=1, padx=5, sticky="w")
    # adis_imu_state_label = tk.Label(sensor_frame, text="ADIS16465", font=normal_font, fg="red")
    # adis_imu_state_label.grid(row=2, column=0, padx=5, sticky="w")
    # ublox_state_label = tk.Label(sensor_frame, text="UBLOX", font=normal_font, fg="red")
    # ublox_state_label.grid(row=2, column=1, padx=5, sticky="w")
    # ylm20_state_label = tk.Label(sensor_frame, text="YLM20", font=normal_font, fg="red")
    # ylm20_state_label.grid(row=3, column=0, padx=5, sticky="w")
    whill_state_label = tk.Label(sensor_frame, text="Whill", font=normal_font, fg="red")
    whill_state_label.grid(row=3, column=1, padx=5, sticky="w")
    
    launch_sensor_button = tk.Button(sensor_frame, text="Launch sensor", command=launch_sensor)
    launch_sensor_button.grid(row=4, column=0, padx=5, sticky="w")
    reset_states_button = tk.Button(sensor_frame, text="Reset", command=reset_states)
    reset_states_button.grid(row=4, column=1, padx=5, sticky="w")
    
    # Localization option section
    T_utm_topomap_file_path_var = tk.StringVar(navigation_menu, value="")
    T_topomap_liomap_file_path_var = tk.StringVar(navigation_menu, value="")
    has_map_var = tk.BooleanVar(navigation_menu, False)
    has_map_checkbox = tk.Checkbutton(navigation_menu, text="Using map", variable=has_map_var, font=normal_font, command=on_localizer_changed)
    has_map_checkbox.grid(row=1, column=0, padx=25, pady=5, sticky="se")
    no_map_var = tk.BooleanVar(navigation_menu, False)
    no_map_checkbox = tk.Checkbutton(navigation_menu, text="No map", variable=no_map_var, font=normal_font, command=on_localizer_changed)
    no_map_checkbox.grid(row=1, column=1, pady=5, sticky="sw")

    # Localizer section
    localizer_frame = tk.Frame(navigation_menu)
    localizer_frame.grid(row=2, column=0, sticky="nw")
    
    localizer_label = tk.Label(localizer_frame, text="--------------  Localizer  --------------", font=bold_font, fg="gray")
    localizer_label.grid(row=0, column=0, columnspan=4, pady=10, sticky="w")
    select_pcd_file_button = tk.Button(localizer_frame, text="Select PCD", state="disabled", command=select_pcd_file)
    select_pcd_file_button.grid(row=1, column=0, sticky="w")
    # select_pcd_file_button.grid_forget()
    select_T_utm_topomap_button = tk.Button(localizer_frame, text="T_u_map", command=select_T_utm_topomap_file)
    select_T_utm_topomap_button.grid(row=1, column=0, sticky="w")
    select_T_utm_topomap_button.grid_forget()
    view_pcd_file_button = tk.Button(localizer_frame, text="View", state="disabled", command=view_pcd)
    view_pcd_file_button.grid(row=1, column=1, sticky="w")
    # view_pcd_file_button.grid_forget()
    select_T_topomap_liomap_button = tk.Button(localizer_frame, text="T_map_lio", command=select_T_topomap_liomap_file)
    select_T_topomap_liomap_button.grid(row=1, column=1, sticky="w")
    select_T_topomap_liomap_button.grid_forget()
    launch_localizer_button = tk.Button(localizer_frame, text="Launch", state="disabled", command=launch_localizer)
    launch_localizer_button.grid(row=1, column=2, sticky="w")
    reset_pcd_path_button = tk.Button(localizer_frame, text="ResetPath", state="disabled", command=reset_pcd_path)
    reset_pcd_path_button.grid(row=1, column=3, sticky="w")
    
    # pcd_file_path_var = tk.StringVar(navigation_menu, "No file path yet")
    pcd_file_path_frame = tk.Frame(localizer_frame)
    # pcd_file_path_canvas = tk.Canvas(pcd_file_path_frame, width=200, height=20)
    # pcd_file_path_canvas.grid(row=0, column=0, sticky="s")
    # pcd_file_path_scrollbar = tk.Scrollbar(pcd_file_path_frame, orient="horizontal", command=pcd_file_path_canvas.xview)
    # pcd_file_path_scrollbar.grid(row=1, column=0)
    # pcd_file_path_canvas.configure(xscrollcommand=pcd_file_path_scrollbar.set)
    # pcd_file_path_label_frame = tk.Frame(pcd_file_path_canvas)
    # pcd_file_path_label_frame.bind("<Configure>", lambda e: pcd_file_path_canvas.configure(scrollregion=pcd_file_path_canvas.bbox("all")))
    # pcd_file_path_label = tk.Label(pcd_file_path_label_frame, textvariable=pcd_file_path_var, font=normal_font)
    # pcd_file_path_label.grid(row=0, column=0, columnspan=3, pady=5, )

    pcd_file_path_scrolltext = scrolledtext.ScrolledText(pcd_file_path_frame, wrap=tk.WORD, width=45, height=3)
    pcd_file_path_scrolltext.grid(row=0, column=0,  columnspan=4)
    pcd_file_path_scrolltext.insert(tk.END, "No file path yet")

    # pcd_file_path_canvas.create_window((0, 0), window=pcd_file_path_label_frame, anchor="nw")
    # pcd_file_path_frame.grid_rowconfigure(0, weight=1)
    # pcd_file_path_frame.grid_columnconfigure(0, weight=1)
    pcd_file_path_frame.grid(row=2, column=0, columnspan=4, pady=5, sticky="w")
    
    # Navigator section
    navigator_frame = tk.Frame(navigation_menu)
    navigator_frame.grid(row=2, column=1, padx=5, sticky="nw")
    
    navigator_label = tk.Label(navigator_frame, text="--------------  Navigator  --------------", font=bold_font, fg="gray")
    navigator_label.grid(row=0, column=0, columnspan=5, pady=10, sticky="w")
    navigator_button_frame = tk.Frame(navigator_frame)
    navigator_button_frame.grid(row=2, column=0, sticky="w")
    select_gridmap_button = tk.Button(navigator_button_frame, text="Select GridMap", state="disabled", command=select_gridmap)
    select_gridmap_button.grid(row=0, column=0, sticky="w")
    view_gridmap_button = tk.Button(navigator_button_frame, text="View", state="disabled", command=view_gridmap)
    view_gridmap_button.grid(row=0, column=1, sticky="w")
    launch_navigator_button = tk.Button(navigator_button_frame, text="Launch", state="disabled", command=launch_navigator)
    launch_navigator_button.grid(row=0, column=2, sticky="w")
    reset_gridmap_path_button = tk.Button(navigator_button_frame, text="ResetPath", state="disabled", command=reset_gridmap_path)
    reset_gridmap_path_button.grid(row=0, column=3, sticky="w")
    # launch_navigator_button = tk.Button(navigator_frame, text="Launch", state="disabled", command=launch_navigator)
    # launch_navigator_button.grid(row=1, column=0, sticky="w")
    use_remap_var = tk.BooleanVar(navigation_menu, False)
    use_hdl_bbs_map_var = tk.BooleanVar(navigation_menu, False)
    navigator_checkbox_frame = tk.Frame(navigator_frame)
    navigator_checkbox_frame.grid(row=1, column=0)
    use_remap_checkbox = tk.Checkbutton(navigator_checkbox_frame, text="use_remap", variable=use_remap_var, font=normal_font, state="disabled", command=on_navigator_changed)
    use_hdl_bbs_map_checkbox = tk.Checkbutton(navigator_checkbox_frame, text="use_hdl_bbs", variable=use_hdl_bbs_map_var, font=normal_font, state="disabled", command=on_navigator_changed)
    use_remap_checkbox.grid(row=0, column=0, sticky="se")
    use_hdl_bbs_map_checkbox.grid(row=0, column=1, sticky="se")

    
    # gridmap_file_path_var = tk.StringVar(navigation_menu, "No file path yet")
    gridmap_file_path_frame = tk.Frame(navigator_frame)
    # gridmap_file_path_canvas = tk.Canvas(gridmap_file_path_frame, width=200, height=20)
    # gridmap_file_path_canvas.grid(row=0, column=0, sticky="s")
    # gridmap_file_path_scrollbar = tk.Scrollbar(gridmap_file_path_frame, orient="horizontal", command=gridmap_file_path_canvas.xview)
    # gridmap_file_path_scrollbar.grid(row=1, column=0)
    # gridmap_file_path_canvas.configure(xscrollcommand=gridmap_file_path_scrollbar.set)
    # gridmap_file_path_label_frame = tk.Frame(gridmap_file_path_canvas)
    # gridmap_file_path_label_frame.bind("<Configure>", lambda e: gridmap_file_path_canvas.configure(scrollregion=gridmap_file_path_canvas.bbox("all")))
    # gridmap_file_path_label = tk.Label(gridmap_file_path_label_frame, textvariable=gridmap_file_path_var, font=normal_font)
    # gridmap_file_path_label.grid(row=0, column=0)
    # gridmap_file_path_canvas.create_window((0, 0), window=gridmap_file_path_label_frame, anchor="nw")
    # gridmap_file_path_frame.grid_rowconfigure(0, weight=1)
    # gridmap_file_path_frame.grid_columnconfigure(0, weight=1)
    # gridmap_file_path_frame.grid(row=2, column=0, columnspan=3, pady=5, sticky="w")
    
    gridmap_file_path_scrolltext = scrolledtext.ScrolledText(gridmap_file_path_frame, wrap=tk.WORD, width=45, height=3)
    gridmap_file_path_scrolltext.grid(row=0, column=0, columnspan=4)
    gridmap_file_path_scrolltext.insert(tk.END, "No file path yet")
    gridmap_file_path_frame.grid(row=3, column=0, columnspan=4, pady=5, sticky="w")
    
    # WhillNavi section
    whill_navi_frame = tk.Frame(navigation_menu)
    whill_navi_frame.grid(row=3, column=0, sticky="nw")
    
    whill_navi_label = tk.Label(whill_navi_frame, text="--------------  WhillNavi  --------------", font=bold_font)
    whill_navi_label.grid(row=0, column=0, columnspan=5, pady=10, sticky="w")
    select_waypoint_button = tk.Button(whill_navi_frame, text="Select Waypoint", command=select_waypoint)
    select_waypoint_button.grid(row=1, column=0, sticky="w")
    view_waypoint_button = tk.Button(whill_navi_frame, text="View", command=view_waypoint)
    view_waypoint_button.grid(row=1, column=1, sticky="w")
    launch_whill_navi_button = tk.Button(whill_navi_frame, text="GO", command=launch_whill_navi)
    launch_whill_navi_button.grid(row=1, column=2, sticky="w")
    reset_waypoint_path_button = tk.Button(whill_navi_frame, text="ResetPath", command=reset_waypoint_path)
    reset_waypoint_path_button.grid(row=1, column=3, sticky="w")

    # waypoint_file_path_var = tk.StringVar(navigation_menu, "No file path yet")
    waypoint_file_path_frame = tk.Frame(whill_navi_frame)
    # waypoint_file_path_canvas = tk.Canvas(waypoint_file_path_frame, width=200, height=20)
    # waypoint_file_path_canvas.grid(row=0, column=0, sticky="s")
    # waypoint_file_path_scrollbar = tk.Scrollbar(waypoint_file_path_frame, orient="horizontal", command=waypoint_file_path_canvas.xview)
    # waypoint_file_path_scrollbar.grid(row=1, column=0)
    # waypoint_file_path_canvas.configure(xscrollcommand=waypoint_file_path_scrollbar.set)
    # waypoint_file_path_label_frame = tk.Frame(waypoint_file_path_canvas)
    # waypoint_file_path_label_frame.bind("<Configure>", lambda e: waypoint_file_path_canvas.configure(scrollregion=waypoint_file_path_canvas.bbox("all")))
    # waypoint_file_path_label = tk.Label(waypoint_file_path_label_frame, textvariable=waypoint_file_path_var, font=normal_font)
    # waypoint_file_path_label.grid(row=0, column=0)
    # waypoint_file_path_canvas.create_window((0, 0), window=waypoint_file_path_label_frame, anchor="nw")
    # waypoint_file_path_frame.grid_rowconfigure(0, weight=1)
    # waypoint_file_path_frame.grid_columnconfigure(0, weight=1)
    # waypoint_file_path_frame.grid(row=2, column=0, columnspan=3, pady=5, sticky="w")
    
    waypoint_file_path_scrolltext = scrolledtext.ScrolledText(waypoint_file_path_frame, wrap=tk.WORD, width=45, height=3)
    waypoint_file_path_scrolltext.grid(row=0, column=0, columnspan=4)
    waypoint_file_path_scrolltext.insert(tk.END, "No file path yet")
    waypoint_file_path_frame.grid(row=2, column=0, columnspan=4, pady=5, sticky="w")

    whill_navi_param_frame = tk.Frame(whill_navi_frame)
    whill_navi_param_frame.grid(row=3, column=0, columnspan=4, sticky="w")
    loop_frame = tk.Frame(whill_navi_param_frame)
    loop_frame.grid(row=0, column=0, sticky="w")
    is_loop_var = tk.BooleanVar(navigation_menu, False)
    is_loop_checkbox = tk.Checkbutton(loop_frame, text="IsLoop", variable=is_loop_var, font=normal_font)
    is_loop_checkbox.grid(row=0, column=0, sticky="w")
    loop_limit_var = tk.IntVar(navigation_menu, 2)
    loop_limit_label = tk.Label(loop_frame, text="LoopLimit: ", font=normal_font)
    loop_limit_label.grid(row=0, column=1, sticky="w")
    loop_limit_entry = tk.Entry(loop_frame, textvariable=loop_limit_var, width=3, font=normal_font)
    loop_limit_entry.grid(row=0, column=2, sticky="w")
    stop_frame = tk.Frame(whill_navi_param_frame)
    stop_frame.grid(row=1, column=0, sticky="w")
    stop_distance_var = tk.DoubleVar(navigation_menu, 1.5)
    stop_distance_label = tk.Label(stop_frame, text="StopDistance: ", font=normal_font)
    stop_distance_label.grid(row=0, column=0, sticky="w")
    stop_distance_entry = tk.Entry(stop_frame, textvariable=stop_distance_var, width=3, font=normal_font)
    stop_distance_entry.grid(row=0, column=1, sticky="w")
    scan_range_frame = tk.Frame(whill_navi_param_frame)
    scan_range_frame.grid(row=2, column=0, sticky="w")
    scan_range_label = tk.Label(scan_range_frame, text="scan_range: ", font=normal_font)
    scan_range_label.grid(row=0, column=0, sticky="w")
    scan_min_range_var = tk.DoubleVar(navigation_menu, -45.0, "scan_min_range")
    scan_max_range_var = tk.DoubleVar(navigation_menu, 45.0, "scan_max_range")
    scan_min_range_var.trace_add(mode="write", callback=check_scan_range)
    scan_max_range_var.trace_add(mode="write", callback=check_scan_range)
    scan_min_range_entry = tk.Entry(scan_range_frame, textvariable=scan_min_range_var, width=5, font=normal_font)
    scan_max_range_entry = tk.Entry(scan_range_frame, textvariable=scan_max_range_var, width=5, font=normal_font)
    range_mark_label = tk.Label(scan_range_frame, text="~", font=bold_font)
    scan_min_range_entry.grid(row=0, column=1, sticky="e")
    range_mark_label.grid(row=0, column=2, sticky="n")
    scan_max_range_entry.grid(row=0, column=3, sticky="w")
    
    # ETC section
    etc_frame = tk.Frame(navigation_menu)
    etc_frame.grid(row=3, column=1)
    rviz_button = tk.Button(etc_frame, text="Rviz2", command=launch_rviz)
    rviz_button.grid(row=0, column=0, padx=5, sticky="e")
    terminate_button = tk.Button(etc_frame, text="TerminateAllRos", command=lambda : terminate_all_ros(process_pids))
    terminate_button.grid(row=0, column=1, padx=5, sticky="w")
    navigation_scrolledtext = scrolledtext.ScrolledText(etc_frame, wrap=tk.WORD, width=45, height=10)
    navigation_scrolledtext.grid(row=1, column=0, columnspan=2, sticky="w")

    adjust_window_size(navigation_menu)
    # set_topmost(navigation_menu)
    navigation_menu.mainloop()

def select_bag_file():
    bag_file_path = filedialog.askdirectory(initialdir=DataPath().bag_dir_path)
    bag_file_path_var.set(bag_file_path)
    splited_path_list = os.path.split(bag_file_path)[0].split("/")
    if len(splited_path_list) >= 7:
        make_dir_dict["place_dir"] = splited_path_list[5]
        make_dir_dict["date_dir"] = splited_path_list[6]
        make_dir_dict["bag_name"] = os.path.split(bag_file_path)[1]
    
    with open(make_dir_yaml_path, "w+") as f:
        write_data = {}
        write_data["make_dir_node"] = {}
        write_data["make_dir_node"]["ros__parameters"] = make_dir_dict
        yaml.safe_dump(write_data, f)
    
    reset_bag_file_path()

def reset_bag_file_path():
    root_log_box.delete("1.0", tk.END)
    root_log_box.insert(tk.END, "Bag file path :" + "\n")
    root_log_box.insert(tk.END, bag_file_path_var.get() + "\n")

def adjust_window_size(root):
    # 更新窗口，确保所有组件都已经被正确计算
    root.update()
    
    # 获取窗口内容所需的宽度和高度
    width = root.winfo_reqwidth()
    height = root.winfo_reqheight()
    
    # 获取屏幕的宽度和高度
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    
    # 计算窗口位置，使其居中
    x = (screen_width - width) // 2
    y = (screen_height - height) // 2
    
    # 设置窗口大小和位置
    root.geometry(f"{width}x{height}+{x}+{y}")

# Main menu
root = tk.Tk()
root.title("WhillNavi2Launcher")
root.geometry("460x200")
icon_path = os.path.join(os.environ["HOME"], ".local", "etc", "WhillNavi2Gui", "icon.png")
icon_image = tk.PhotoImage(file=icon_path)
root.iconphoto(True, icon_image)

make_dir_yaml_path = get_file_path("whill_navi2", "make_dir_node_params.yaml")
ros2_whill_yaml_path = get_file_path("whill_navi2", "ros2_whill_params.yaml")
make_dir_dict = get_param_dict(make_dir_yaml_path, "make_dir_node")
whill_joy_dict = get_param_dict(ros2_whill_yaml_path, "whill_joy")
bold_font = font.Font(family="Helvetica", size=16, weight="bold")
normal_font = font.Font(family="Helvetica", size=12, weight="normal")
slice_number = 1

main_button_frame = tk.Frame(root)
main_button_frame.grid(row=0, column=0, sticky="nw")
data_gather_button = tk.Button(main_button_frame, text="DataGather", command=data_gather)
data_gather_button.grid(row=0, column=0, padx=5, pady=5, sticky="w")
glim_offline_button = tk.Button(main_button_frame, text="GlimOffline", command=glim_offline)
glim_offline_button.grid(row=0, column=1, padx=5, pady=5, sticky="w")
hdl_waypoint_button = tk.Button(main_button_frame, text="HdlWaypoint", command=hdl_waypoint)
hdl_waypoint_button.grid(row=0, column=2, padx=5, pady=5, sticky="w")
navigation_button = tk.Button(main_button_frame, text="Navigation", command=navigation)
navigation_button.grid(row=0, column=3, padx=5, pady=5, sticky="w")

bag_frame = tk.Frame(root)
bag_frame.grid(row=1, column=0, pady=5, sticky="nw")
bag_file_path_button = tk.Button(bag_frame, text="Bag", command=select_bag_file)
bag_file_path_button.grid(row=0, column=0, padx=10, sticky="w")

bag_file_path_var = tk.StringVar(root, "No bag path yet")
# bag_file_path_frame = tk.Frame(root)
# bag_file_path_canvas = tk.Canvas(bag_file_path_frame, width=300, height=20)
# bag_file_path_canvas.grid(row=0, column=0, sticky="w")
# waypoint_file_path_scrollbar = tk.Scrollbar(bag_file_path_frame, orient="horizontal", command=bag_file_path_canvas.xview)
# waypoint_file_path_scrollbar.grid(row=1, column=0)
# bag_file_path_canvas.configure(xscrollcommand=waypoint_file_path_scrollbar.set)
# waypoint_file_path_label_frame = tk.Frame(bag_file_path_canvas)
# waypoint_file_path_label_frame.bind("<Configure>", lambda e: bag_file_path_canvas.configure(scrollregion=bag_file_path_canvas.bbox("all")))
# waypoint_file_path_label = tk.Label(waypoint_file_path_label_frame, textvariable=bag_file_path_var, font=normal_font)
# waypoint_file_path_label.grid(row=0, column=0)
# bag_file_path_canvas.create_window((0, 0), window=waypoint_file_path_label_frame, anchor="nw")
# bag_file_path_frame.grid_rowconfigure(0, weight=1)
# bag_file_path_frame.grid_columnconfigure(0, weight=1)
# bag_file_path_frame.grid(row=1, column=0, padx=60, sticky="w")

root_log_box = scrolledtext.ScrolledText(root, wrap=tk.WORD, width=50, height=5)
root_log_box.grid(row=2, column=0, columnspan=4, padx=15, pady=10, sticky="w")
root_log_box.insert(tk.END, "Bag file path :" + "\n")
root_log_box.insert(tk.END, bag_file_path_var.get() + "\n")

adjust_window_size(root)
# set_topmost(root)

# 启动主循环
root.mainloop()
