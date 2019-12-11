cd ~
# Adicionar no final do .bashrc as seguintes linhas
# Se for PC:

#Endereço e porta do mestre (PC)
echo "export ROS_MASTER_URI=http://localhost:11311/" >> .bashrc
# Ip do mestre na rede pego com ifconfig
echo "export ROS_HOSTNAME=192.168.0.104" >> .bashrc
echo "export ROS_IP=192.168.0.104" >> .bashrc

# Se for rasp, descomentar:
##Endereço e porta do mestre (RP)
#echo "export ROS_MASTER_URI=http://192.168.0.104:11311/" >> .bashrc
## Ip do escravo na rede pego com ifconfig
#echo "export ROS_HOSTNAME=192.168.0.109" >> .bashrc
#echo "export ROS_IP=192.168.0.109" >> .bashrc

source .bashrc
