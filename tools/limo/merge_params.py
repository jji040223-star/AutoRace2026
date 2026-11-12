import os, sys, yaml
d = yaml.safe_load(open(sys.argv[1]))
d["lane_follower"]["ros__parameters"]["linear_speed"] = float(sys.argv[3])
yaml.safe_dump(d, open(sys.argv[2], "w"), allow_unicode=True)
