# Launch file functions
bringup_name=amigo
package_path=`rospack find ${bringup_name}_bringup`
launch_files=$(find ${package_path}/launch -name '*.launch')

install_services_cmd=""
for launch_file in ${launch_files}; do
    launch_file_name=$(echo ${launch_file##*/} | cut -d"." -f1)
    name=${bringup_name//_/-}-${launch_file_name//_/-}
    eval "${name}() { export ROBOT_BRINGUP_PATH=${package_path} && roslaunch ${launch_file} \$@; }"

    install_services_cmd="$install_services_cmd && echo 'Creating /etc/systemd/user/$name.service' && rosrun ${bringup_name}_bringup systemd_service_from_launch_file ${launch_file} amigo ~/.tue/setup.bash | sudo tee /etc/systemd/system/$name.service"
done

eval "${bringup_name}-install-services() { echo 'Installing systemd services for ${bringup_name} ...' $install_services_cmd; }"
