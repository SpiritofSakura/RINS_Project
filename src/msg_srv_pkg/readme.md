## How to use this package
add a meesage blueprint (mymsg.msg) to msg folder or
add a service blueprint (mysrv.srv) to srv folder

then in CMakeLists.txt you have to tell it to generate those interfaces

for example this was how it was done in dn2
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Dn2.msg"
  "srv/Dn2.srv"
)
