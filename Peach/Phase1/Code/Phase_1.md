## Phase1：基础工具与环境

### 任务1.1： Linux 命令行基础

- #### 进入目录：

  ```bash
  cd Study_Startup_Diary
  ```

  反馈：

  ```
  mxy@orangepi5pro:~/Study_Startup_Diary$ 
  ```

- #### 查看文件列表：

  ```bash
  ls
  ```

  反馈：

  ```
  Assignments  Belief  Fish  Peach  README.md  Resources  Sheep
  ```

- #### 确认当前位置：

  ```bash
  pwd
  ```

  反馈：

  ```
  /home/mxy/Study_Startup_Diary
  ```

- #### 创建和删除目录

  ```bash
  #创建空白目录
  mkdir practice1
  mkdir practice2
  #查看
  ls
  ```
  
  反馈：
  
  ```
  Desktop    Downloads  Pictures   practice2  Study_Startup_Diary  Videos
  Documents  Music      practice1  Public     Templates
  ```

- #### 创建和复制文件

  ```bash
  #创建空白文件
  touch 111.txt
  #复制到practice1
  cp 111.txt practice1/
  #查看
  ls practice1
  ```
  
  反馈：
  
  ```
  111.txt
  ```

- #### 移动和重命名文件

  ```bash
  #移动并更改文件名
  mv 111.txt practic2/112.txt
  #验证是否从主目录中消失
  ls
  #验证是否在practice2中
  ls practice2
  ```

  反馈：

  ```
  Desktop    Downloads  Pictures   practice2  Study_Startup_Diary  Videos
  Documents  Music      practice1  Public     Templates
  
  112.txt
  ```

- #### 删除文件和目录

  ```bash
  #删除practice1目录里的文件
  rm practice1/111.txt
  #删除practice1（空白）目录
  rmdir practice1
  #删除整个目录及文件
  rm -r practice2
  #查看
  ls
  ```

  反馈：

  ```
  Desktop    Downloads  Pictures  Study_Startup_Diary  Videos
  Documents  Music      Public    Templates
  ```

- #### 使用nano编译器

  ```bash
  #用nano创建并编辑一个空白文件
  nano 113.txt
  #ctrl+O保存
  #ctrl+x退出
  ```

- #### 查看环境变量

  ```bash
  echo $PATH
  ```

  反馈：

  ```
  /usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin
  ```

- #### 临时变量设置

  ```bash
  #设置临时变量
  export practice="Temporary Value"
  #验证
  echo $practice
  ```

  反馈：

  ```
  Temporary Value
  ```

### 任务1.2 ：Linux系统设置（中国区）

- #### 更换国内镜像源（提高下载速度）

```bash
#系统版本确认
cat /etc/os-release
```

反馈：

```
10-wifi-disable-powermanagement.rules  84-audio-orangepi.rules
40-usb_modeswitch.rules                88-rockchip-camera.rules
50-usb-realtek-net.rules               90-naming-audios.rules
60-drm.rules                           90-pulseaudio-rockchip.rules
83-typec.rules                         99-rockchip-permissions.rules
mxy@orangepi5pro:~$ cat /etc/os-release
NAME="Ubuntu"
VERSION="20.04.6 LTS (Focal Fossa)"
ID=ubuntu
ID_LIKE=debian
PRETTY_NAME="Ubuntu 20.04.6 LTS"
VERSION_ID="20.04"
HOME_URL="https://www.ubuntu.com/"
SUPPORT_URL="https://help.ubuntu.com/"
BUG_REPORT_URL="https://bugs.launchpad.net/ubuntu/"
PRIVACY_POLICY_URL="https://www.ubuntu.com/legal/terms-and-policies/privacy-policy"
VERSION_CODENAME=focal
UBUNTU_CODENAME=focal
```

确认为Ubuntu20.04，focal

输入命令：

```bash
sudo nano /etc/apt/sources.list
```

在阿里官网找到对应版本代码并替换

```
deb https://mirrors.aliyun.com/ubuntu/ focal main restricted universe multiverse
deb-src https://mirrors.aliyun.com/ubuntu-ports/ focal main restricted universe multiverse

deb https://mirrors.aliyun.com/ubuntu/ focal-security main restricted universe multiverse
deb-src https://mirrors.aliyun.com/ubuntu-ports/ focal-security main restricted universe multiverse

deb https://mirrors.aliyun.com/ubuntu/ focal-updates main restricted universe multiverse
deb-src https://mirrors.aliyun.com/ubuntu/ focal-updates main restricted universe multiverse

#deb https://mirrors.aliyun.com/ubuntu/ focal-proposed main restricted universe multiverse
#deb-src https://mirrors.aliyun.com/ubuntu/ focal-proposed main restricted universe multiverse

deb https://mirrors.aliyun.com/ubuntu/ focal-backports main restricted universe multiverse
deb-src https://mirrors.aliyun.com/ubuntu/ focal-backports main restricted universe multiverse
```

反馈failed，有些not founded，问Gemini，说香橙派为` ARM `架构，因此需要使用 `ubuntu-ports` 镜像源。

将代码中原来的`/ubuntu/`全部替换成`/ubuntu-ports/`

再输入命令`/sudo apt update/`,成功。

- #### 学习`sudo`


```bash
#查看 /etc/sudoers 文件的内容
sudo cat /etc/sudoers
```

反馈：

```
#
# This file MUST be edited with the 'visudo' command as root.
#
# Please consider adding local content in /etc/sudoers.d/ instead of
# directly modifying this file.
#
# See the man page for details on how to write a sudoers file.
#
Defaults        env_reset
Defaults        mail_badpass
Defaults        secure_path="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:                                                                                                     /sbin:/bin:/snap/bin"

# Host alias specification

# User alias specification

# Cmnd alias specification

# User privilege specification
root    ALL=(ALL:ALL) ALL

# Members of the admin group may gain root privileges
%admin ALL=(ALL) ALL

# Allow members of group sudo to execute any command
%sudo   ALL=(ALL:ALL) ALL

# See sudoers(5) for more information on "#include" directives:

#includedir /etc/sudoers.d
orangepi ALL=(ALL) NOPASSWD: /usr/bin/psd-overlay-helper

```

- #### 串口权限设置


学习`groups`命令

```bash
groups
```

反馈：

```
mxy sudo
#用户mxy属于sudo用户组
```

将当前用户添加到 `dialout` 组

```bash
sudo usermod -aG dialout $USER
#重新登陆后
groups
```

反馈：

```
mxy dialout sudo
```

- #### udev规则学习


查看 `sl_sensor` 提供的 `udev_rules` 目录下的示例规则文件

示例：udev_rules/98-versa-vis.rules

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="824d", SYMLINK+="versavis", GROUP="dialout", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="804d", SYMLINK+="versavis", GROUP="dialout", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="004d", SYMLINK+="versavis", GROUP="dialout", MODE="0666"
```

| **关键字**                 | **语法**                                                     |
| :------------------------- | ------------------------------------------------------------ |
| `SUBSYSTEM=="tty"`         | 匹配： 规则只适用于 串口设备(`tty` 子系统，如 `/dev/ttyUSB0` )。 |
| `ATTRS{idVendor}=="2341"`  | 匹配： 厂商 ID 必须是 **`2341`**。                           |
| `ATTRS{idProduct}=="8244"` | 匹配： 产品 ID 必须是 **`8244`**。                           |
| `SYMLINK+="versavis"`      | 赋值： 创建一个固定别名：`/dev/versavis`。                   |
| `GROUP="dialout"`          | 赋值： 将设备文件的用户组设置为 **`dialout`**。              |
| `MODE="0666"`              | 赋值： 设置权限为 `0666` (允许所有用户和程序读写该设备)。    |

| **关键字** | **作用**                                 | **痛点 (问题)**                                              |
| ---------- | ---------------------------------------- | ------------------------------------------------------------ |
| `SYMLINK`  | 创建固定别名（如 `SYMLINK+="sensor_A"`） | 名称不固定。 每次插入 USB 设备时，设备名可能变：`/dev/ttyUSB0` 可能变成 `/dev/ttyUSB1`，导致访问一些程序失败或找不到 |
| `GROUP`    | 设置用户组（如 `GROUP="dialout"`）       | 权限受限。 默认情况下，`/dev` 下的设备文件只允许 `root` 用户和特定的组访问。 |
| `MODE`     | 设置读写权限（如 `MODE="0666"`）         | 权限不灵活。 默认权限可能不允许程序写入设备。                |

### 任务 1.3: Linux Shell 脚本 (`setup_project.sh`)

- #### 编写脚本

  ```
  #!/bin/bash
  
  PROJECT_NAME="my_sl_project"
  
  if [ -d "$PROJECT_NAME" ]; then
      echo "error:existed。"
      exit 1
  fi
  
  mkdir -p "$PROJECT_NAME"/src
  mkdir -p "$PROJECT_NAME"/include
  mkdir -p "$PROJECT_NAME"/build
  touch "$PROJECT_NAME"/README.md
  
  echo "Successed!"
  ```
  
  赋予执行权限
  
  ```bash
  chmod +x setup_project.sh
  ```
  
  运行脚本
  
  ```bash
  ./setup_project.sh
  ```
  
  反馈
  
  ```bash
  Successed！
  ```
  
  检查
  
  ```bash
  cd my_sl_project
  ls
  #反馈：
  build  include  README.md  src
  ```
  
  ### 任务 1.4: Git 版本控制与 GitHub 协作
  
- #### 在 GitHub 上创建新仓库 `my_s1_project`

- #### 在本地初始化 Git 仓库

  ```bash
  cd my_s1_project
  git init
  ```

- #### 创建至少 3 次有意义的提交

  ```bash
  #1.暂存目录下所有文件包
  git add .
  #保存到本地仓库
  git commit -m "feat: Initialize project struture with src, include, build"
  #2.新建一个代码文件
  touch src/main.cpp
  #暂存
  git add src/main.cpp
  #保存到本地仓库
  git commit -m "Add initial main C++ file"
  #3.README.md添加内容
  nano README.md 
  git add README.md
  git commit -m "Add initial README.md"
  ```

  远程连接仓库(用ssh密钥，不知道为什么https不太好使，说找不到)

  ```bash
  git remote add origin git@github.com:xxyzzz-mxy/my_s1_project.git 
  ```

  首次推送至远程

  ```bash
  git push -u origin main
  ```

- #### 创建并切换到一个新分支

  ```bash
  git checkout -b feature/new-task
  ```

  创建空白文件并推送至新分支中

  ```bash
  touch config.h
  git add config.h
  git commit -m "Add in new branch"
  git push -u origin feature/new-task
  ```
  
  返回main
  
  ```bash
  git checkout main
  ```
  
  
