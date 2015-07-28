
echo
echo I am assuming you have ROS installed...
echo

echo
echo Installing Pip
echo
sudo apt-get install python-pip

echo
echo Installing Pip requirements.txt
echo
sudo pip install -r requirements.txt

echo 
echo Installing rospy_message_converter
echo
mkdir resources;
cd resources; git clone https://github.com/baalexander/rospy_message_converter.git; cd -;
cd resources/rospy_message_converter; sudo python setup.py install; cd -;

echo
echo Installing ZeroMQ-Ros
echo
cd src; sudo python setup.py install; cd -;
