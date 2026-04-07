#!/usr/bin/env python3
from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SO101Follower

config = SO101FollowerConfig(
    port='/dev/ttyACM2',
    id='official_arm',
)

follower = SO101Follower(config)
follower.setup_motors()
