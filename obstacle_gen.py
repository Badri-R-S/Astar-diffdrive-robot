

import pygame
import math
import numpy as np
def pathu():
    # Initializing Pygame
    pygame.init()

    # Initializing surface
    surface = pygame.display.set_mode((600,250))
    surface.fill((0,0,0))

    # Initializing Color
    color = pygame.Color("Yellow")
    HexV = []
    HexV1=[]
    center = [300,125]
    side_length =65
    side_length1=75
    Trianlge=[(460,25),(460,225),(510,125)]
    Trianlge1=[(455,10),(455,240),(520,125)]
    for i in range(6):
        angle_deg = 60 * i - 60
        angle_rad = math.pi / 180 * angle_deg
        x = center[0] + side_length1 * math.cos(angle_rad)
        y = center[1] + side_length1 * math.sin(angle_rad)
        x1 = center[0] + side_length * math.cos(angle_rad)
        y1 = center[1] + side_length * math.sin(angle_rad)
        HexV.append((x1,y1))
        HexV1.append((x,y))

    print(HexV1)
    # # Initialise clearance of 5mm
    obs=[]
    #initialize Borders

    color1 = (255,255,255)
    pygame.draw.rect(surface, color1, (0,0,5,250))
    pygame.draw.rect(surface, color1, (595,0,5,250))
    pygame.draw.rect(surface, color1, (0,0,600,5))
    pygame.draw.rect(surface, color1, (0,245,600,5))



    obs.append(pygame.draw.rect(surface, color1, pygame.Rect(100-5, 145-5, 50+10, 100+10)))
    obs.append(pygame.draw.rect(surface, color1, pygame.Rect(100-5, 5-5, 50+10, 100+10)))
    obs.append(pygame.draw.polygon(surface,color1,HexV1))
    obs.append(pygame.draw.polygon(surface,color1,Trianlge1))


    #initialize Shapes
    
    obs.append(pygame.draw.rect(surface, color, pygame.Rect(100, 145, 50, 100)))
    obs.append(pygame.draw.rect(surface, color, pygame.Rect(100, 5, 50, 100)))
    obs.append(pygame.draw.polygon(surface,color,HexV))
    obs.append(pygame.draw.polygon(surface,color,Trianlge))

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False  
        pygame.display.flip()
    pygame.quit()
    return surface
def obs_coord(x,y):
    occupied_coords = []
    ind=0
    #Bounds
    if ((x<5) or (x>595)) or ((y<5) or (y>245)):
        occupied_coords.append([x,y])
        ind=1
    #Hexagon 
    if (x >= 231) and (x <= 369) and (y >= 15.0 + 0.577*(x-260)) and (y <= 232.5 - 0.577*(x-260)) and (y <= 230.5 + 0.577*(x-325.5)) and (y >= 20.0 - 0.577*(x-325.5)):
        occupied_coords.append([x,y])
        ind=1
    #Rectangle
    if (95 <= x  and x <= 155 and 0 <= y and  y<= 110):
        occupied_coords.append([x,y])
        ind=1
    #Rectangle
    if (95 <= x <= 155 and 140 <= y and  y<= 250):
        occupied_coords.append([x,y])
        ind=1
    #triangle
    if (x>=455)and (-y+10 <= (-23/13)*(x - 455)) and (-y+240 >= (23/13)*(x-455))  :
        occupied_coords.append([x,y])
        ind=1
    return ind
# pathu()