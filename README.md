# sa_moving_sensor

This repository integrates multi target tracking algorithm with graphic inference of trajectories to build the situational awareness.

## Structure

The structure is roughly the following:

Observation -> MTT -> estimated trajectories -> Inference -> ?

## Inputs

1. The input format to MTT is a list of positions, such as 

`z_k = [[x1, y1], [x2, y2], ...]`

2. The input format to Inference is a

## Outputs 

1. The output format from MTT is a list of estimated trajectories, i.e.

`est_list_k = [[id1, [x1, y1], P1], [id2, [x2, y2], P2], ...] `
