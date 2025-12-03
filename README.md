首先要把两个文件夹放在一个文件夹里面，该文件夹命名为dummy_ws;
一定遵循指令打开顺序;
先打开gazebo;
再打开Rviz（Rviz会自动读取Gazebo里面的状态）;
由于gazebo打开，机械臂会自动掉落，很正常，这一点在终端二的代码中已经纠正;
在Rivz里面实行拖动就行;
主要是plan-execute-plan-execute;
其中如果出现拖动不顺利，可以换个角度拖动;
