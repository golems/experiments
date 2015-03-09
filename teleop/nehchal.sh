# @author: Nehchal J.
# Mar 5, 2015

#remove channels
ach rm krang_lgrip_waypts
ach rm krang_rgrip_waypts
ach rm krang_lgrip_pose
ach rm krang_rgrip_pose 
ach rm krang_lgrip_ft
ach rm krang_rgrip_ft
ach rm krang_body_state
ach rm krang_lgrip_ref_vel
ach rm krang_rgrip_ref_vel



# Make the channels required by workspaced
ach mk krang_lgrip_waypts -o 666
ach mk krang_rgrip_waypts -o 666
ach mk krang_lgrip_pose -o 666
ach mk krang_rgrip_pose -o 666
ach mk krang_lgrip_ft -o 666
ach mk krang_rgrip_ft -o 666
ach mk krang_body_state -o 666
ach mk krang_lgrip_ref_vel -o 666
ach mk krang_rgrip_ref_vel -o 666

