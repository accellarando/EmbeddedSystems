# Commands
Ella Moss, Ryan Lam, Matt Alter - lightcyan team

## Purpose
This document details the communication protocol for our final project.

## Command format
Each command will consist of one or two bytes, followed by a '\\n' stop bit. "Vector" commands will consist of two bytes, representing the direction and magnitude of the requested motion. "Scalar" commands will consist of only one byte.

## Vector commands

| Command | Description | Key |
| --- | --- | --- |
| Right | Turn right the specified amount, in 20-degree increments, represented by a digit 1-9. Note that the number '4' will turn 90 degrees rather than 80. | 'd' |
| Left | Turn left the specified amount, in 20-degree increments, represented by a digit 1-9. Note that the number '4' will turn 90 degrees rather than 80. | 'a' |
| Forward | Go forward the specified amount, in 20-centimeter increments, represented by a digit 0-9. 0 means "continue forward indefinitely." Interrupted if a box is detected. | 'w' |

## Scalar commands

| Command | Description | Key |
| --- | --- | --- |
| Proceed | Originally, intended to return to autonomous mode, continue forward and navigate around boxes while clear and heading is (-90, 90) degrees. Not implemented for this project. | 'p' |
| Log | Log sensor data back to the controller. | 'l' |
| Kill | Stop all motors. | 'x' |
