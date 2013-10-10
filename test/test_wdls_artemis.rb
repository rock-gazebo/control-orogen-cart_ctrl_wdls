require 'orocos'
require 'vizkit'

if !ARGV[0] or !ARGV[1] or !ARGV[2]
    STDERR.puts "usage: ruby test.rb <cart_ctrl_config.yml> <solver_config.yml> <driver_config.yml> <x> <y> <z> <qx> <qw> qz> <qw>"
    STDERR.puts "((x,y,z)(qx,qy,qz,qw): Goal pose wrt root frame. position in [m], orientation as Quaternion."
    STDERR.puts "If no pose values are given, default pose will be used"
    exit 1
end

Orocos.initialize

cart_ctrl_config = ARGV[0] if ARGV[0]
solver_config = ARGV[1] if ARGV[1]
driver_config = ARGV[2] if ARGV[2]

x = ARGV[3].to_f if ARGV[3] 
y = ARGV[4].to_f if ARGV[4]
z = ARGV[5].to_f if ARGV[5] 
qx = ARGV[6].to_f if ARGV[6] 
qy = ARGV[7].to_f if ARGV[7] 
qz = ARGV[8].to_f if ARGV[8] 
qw = ARGV[9].to_f if ARGV[9] 

# Default pose
if x.nil? or y.nil? or z.nil? or qx.nil? or qy.nil? or qz.nil? or qw.nil?
   x = 0.3 
   y = 0.0
   z = 0.5
   qx = qy = qz = 0.0 
   qw = 1.0
end

Orocos.run 'cart_ctrl_wdls::CartCtrl' => 'controller',
           'cart_ctrl_wdls::WDLSSolver' => 'solver',
           'fake_joint_driver::Task' => 'driver',
           'cart_ctrl_wdls::ToPosConverter' => 'converter' do  
    
    controller = Orocos.name_service.get 'controller'
    solver = Orocos.name_service.get 'solver'
    driver = Orocos.name_service.get 'driver'
    converter = Orocos.name_service.get 'converter'

    controller.apply_conf_file(cart_ctrl_config)
    solver.apply_conf_file(solver_config)
    driver.apply_conf_file(driver_config)

    solver.cartesian_status.connect_to controller.cartesian_status
    controller.ctrl_out.connect_to solver.desired_twist
    converter.command.connect_to solver.solver_output

    #
    # These might have to be changed if using real robot
    # 
    driver.joint_status.connect_to solver.joint_status
    driver.command.connect_to converter.command_out    
    driver.joint_status.connect_to converter.joint_status  

    controller.configure
    solver.configure
    driver.configure
    solver.start 
    controller.start 
    driver.start 
    converter.start

    writer_command = controller.command.writer 

    command = Types::Base::Samples::RigidBodyState.new
    command.position = Types::Base::Vector3d.new(x,y,z)
    command.orientation = Types::Base::Quaterniond.new(qw, qx, qy, qz)

    writer_command.write(command)
     
    Vizkit.exec

end
