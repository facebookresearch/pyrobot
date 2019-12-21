import Pyro4

if __name__ == '__main__':
	import argparse

	parser = argparse.ArgumentParser(description='Pass in server device IP')
    parser.add_argument('--ip', help='Server device (robot) IP. Default is 192.168.0.0', type=str, default="192.168.0.0")

    args = parser.parse_args()

	bot = Pyro4.Proxy("PYRONAME:remotelocobot@" + args.ip)

	# call functions from server wrapper
	pan_angle = bot.get_pan()
	tilt_angle = bot.get_tilt()
	bot.set_pan_tilt(pan_angle + 0.1, tilt_angle - 0.1)

	rgb = bot.get_rgb()
	depth = bot.get_depth()

	# if using byte serialization for image:
	import numpy as np
	import base64
	rgb_bytes = base64.b64decode(bot.get_rgb_bytes()['data'])
	img = np.frombuffer(rgb_bytes, dtype=np.int64).reshape(480, 640, 3)
