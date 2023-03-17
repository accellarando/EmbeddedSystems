# Commands
Ella Moss, Ryan Lam, Matt Alter - lightcyan team

## Purpose
This document details the communication protocol for our final project.

## Command format
Each command will consist of one or two bytes, followed by a '\\n' stop bit. "Vector" commands will consist of two bytes, representing the direction and magnitude of the requested motion. "Scalar" commands will consist of only one byte.

## Vector commands

| Command | Description | Key |
| --- | --- | --- |
| Right | Turn right the specified amount, in 20-degree increments, represented by a digit 1-9. | 'd' |
| Left | Turn left the specified amount, in 20-degree increments, represented by a digit 1-9. | 'a' |
| Forward | Go forward the specified amount, in inches, represented by a digit 1-9. Interrupted if a box is detected. | 'w' |

## Scalar commands

| Command | Description | Key |
| --- | --- | --- |
| Proceed | Return to autonomous mode, continue forward and navigate around boxes while clear and heading is (-180, 180) degrees. | 'p' |
| Log | Log sensor data back to the controller. | 'l' |
