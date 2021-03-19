import yaml
import os
import sys
import imp
import rospkg

em = None
for path in sys.path:
    filename = os.path.join(path, 'em.py')
    if os.path.exists(filename):
        em = imp.load_source('em', filename)
        if "expand" in dir(em):
            break
# For-else: else is called if loop doesn't break
else:
    print(
        "ERROR: could not find module em, please sudo apt install python-empy")
    exit(2)


if __name__ == '__main__':

    r = rospkg.RosPack()
    defaultCfgPath = os.path.join(r.get_path('stag_ros'), "cfg/single_config.yaml")

    while True:
        try:
            cfgPath = raw_input("Enter the path to the single_config.yaml (or leave empty for default - %s): --> " % defaultCfgPath)
            if not cfgPath:
                cfgPath = defaultCfgPath
            if not os.path.isfile(cfgPath):
                print("Sorry, there is no file on specified path.")
                continue
        except (ValueError, NameError, SyntaxError):
            print("Sorry, your input was wrong.")
            continue
        else:
            break

    while True:
        try:
            firstId = raw_input("Enter the first id (or leave empty for default - 1): --> ")
            if not firstId:
                firstId = 1
            else:
                firstId = int(firstId)
        except (ValueError, NameError, SyntaxError):
            print("Sorry, your input was wrong.")
            continue
        else:
            break

    while True:
        try:
            lastId = input("Enter the last id: --> ")
        except (ValueError, NameError, SyntaxError):
            print("Sorry, your input was wrong.")
            continue
        else:
            break

    while True:
        try:
            markerSize = float(raw_input("Enter the size of markers in meters: --> ").replace(",", "."))
        except (ValueError, NameError, SyntaxError):
            print("Sorry, your input was wrong.")
            continue
        else:
            break

    halfMarkerSize = markerSize / 2.0

    inner = ""

    for i in range(firstId, lastId + 1):
        s = em.expand("""    {
      frame: "tag_@(id)",
      id: @(id),
      corners: [
        [-@(halfMarkerSize), @(halfMarkerSize), 0.0],
        [@(halfMarkerSize), @(halfMarkerSize), 0.0],
        [@(halfMarkerSize), -@(halfMarkerSize), 0.0]
      ]
    },
""", {"id": i, "halfMarkerSize": halfMarkerSize})

        inner += s

    inner = inner[:-2]  # remove final ",\n"

    out = em.expand("""tags:
  [
@(str)
  ]

# Marker corner order:
# 1          2
#  HD########
#  #ID#  ####
#  ##  ##  ##
#  ####  ####
#  ##########
# 4          3

    """, {"str": inner})

    f = open(defaultCfgPath, "w")
    f.write(out)
    f.close()
