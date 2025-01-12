#!/usr/bin/env python3

'''
Resources:
- https://www.latlong.net
- https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
'''

from argparse import ArgumentParser
import math
import os
import subprocess

def main():
    parser = ArgumentParser(description="converts lon/lat to tile numbers")
    parser.add_argument("--lon", help="longitude", type=float, required=True)
    parser.add_argument("--lat", help="latitude", type=float, required=True)
    parser.add_argument("--zoom", help="zoom level (0-19)", type=int, default=16)
    parser.add_argument("--blocks", help="number of additional adjacent tiles", type=int, default=0)
    parser.add_argument("-o", "--output", help="output tile path", type=str, default="./tile")
    parser.add_argument("-d", "--download", help="True if downloading tiles", type=bool, default=False)
    args = parser.parse_args()

    n = 2 ** args.zoom
    xtile = int(n * ((args.lon + 180) / 360))
    ytile = int(n * (1 - (math.log(math.tan(math.radians(args.lat)) + 1 / math.cos(math.radians(args.lat))) / math.pi)) / 2)
    print(f"xtile: {xtile}")
    print(f"ytile: {ytile}")
    print(f"openstreetmap link: https://tile.openstreetmap.org/{args.zoom}/{xtile}/{ytile}.png")

    if args.download:
        init_path = os.getcwd()
        for i in range(-args.blocks, args.blocks + 1):
            subprocess.run(["mkdir", "-p", os.path.join(args.output, str(args.zoom), str(xtile + i))])
            os.chdir(os.path.join(args.output, str(args.zoom), str(xtile + i)))
            for j in range(-args.blocks, args.blocks + 1):
                subprocess.run([
                    "curl", "-O", f"https://tile.openstreetmap.org/{args.zoom}/{xtile + i}/{ytile + j}.png"
                ])
            os.chdir("../../..")
        os.chdir(init_path)
                
if __name__ == "__main__":
    main()
