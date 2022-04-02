#!/usr/bin/env python3

import pygame
from pygame.locals import *
import sys
import time
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 4242

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

sock.bind(("127.0.0.1", 42067))

pygame.init()

window_width = 500
window_height = 500

screen = pygame.display.set_mode((window_width, window_height))
pygame.display.set_caption('Move leg with mouse')
font = pygame.font.SysFont(None, 24)

WHITE = pygame.Color(255,255,255)

while True:
    for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

    screen.fill((0, 0, 0))
    
    x,y = pygame.mouse.get_pos()

    # Map to +/-0.3m in X and Y
    scaled_x = ((x - window_width / 2) / window_width) * 2 * 0.3
    scaled_y = ((y - window_height / 2) / window_height) * 2 * 0.3 * -1

    message = "{scaled_x}|{scaled_y}".format(scaled_x=scaled_x, scaled_y=scaled_y)
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

    pygame.draw.circle(screen, WHITE, (x,y), 10, 10)

    x_text = font.render("X: {}".format(x), True, WHITE)
    y_text = font.render("Y: {}".format(y), True, WHITE)
    screen.blit(x_text, (20, 20))
    screen.blit(y_text, (20, 40))

    scaled_x_text = font.render("Scaled X: {}m".format(scaled_x), True, WHITE)
    scaled_y_text = font.render("Scaled Y: {}m".format(scaled_y), True, WHITE)
    screen.blit(scaled_x_text, (20, 60))
    screen.blit(scaled_y_text, (20, 80))

    pygame.display.update()
    time.sleep(1/120)