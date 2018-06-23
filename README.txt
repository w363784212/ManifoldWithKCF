
Manifold获取M100 X3图传代码，并利用KCF跟踪图像中物体

# 运行前确保Manifold系统时间正确，不正确则 sudo ntpdate -u ntp.api.bz 密码ubuntu

.pro为QT工程文件, qmake -makefile命令由.pro生成makefile文件  qmake -project由makefile生成.pro文件

生成makefile文件后, sudo make， 生成可执行文件 
 
然后 sudo ./Manifold-Cam-Master -[dgt]   -d --display image   -g --getImageBuffer   -t --transport image to mobile app

其中d和g不能同时用，一般用-t或者-gt  (-g时会利用KCF跟踪图像中目标)

注意：该程序一定要在命令行中用上述方法运行，并且退出时一定要按ctrl+c等待退出，
      
      如果在QT中运行，则无法正常退出，导致资源没有被正常释放，则之后程序无法再正常运行，只能重启Manifold才行。