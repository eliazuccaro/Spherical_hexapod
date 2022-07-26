mkdir -p ~/sources
git clone https://github.com/vitiral/gpio.git
cd gpio
python3 setup.py build
sudo python3 setup.py install
