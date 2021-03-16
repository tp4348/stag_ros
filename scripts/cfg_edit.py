import yaml
import os
import sys
import imp

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

defaultCfgPath = "../cfg/single_config.yaml"

if __name__ == '__main__':
    while True:
        try:
            cfgPath = str(raw_input("Enter the path to the single_config.yaml (or leave empty for default - %s): --> " % defaultCfgPath))
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
            numMarkers = input("Enter the number of markers: --> ")
        except (ValueError, NameError, SyntaxError):
            print("Sorry, your input was wrong.")
            continue
        else:
            break

    while True:
        try:
            markerSize = input("Enter the size of markers in meters: --> ")
        except (ValueError, NameError, SyntaxError):
            print("Sorry, your input was wrong.")
            continue
        else:
            break


    halfMarkerSize = markerSize / 2.0

    inner = ""

    for i in range(numMarkers + 1): # Sometimes first marker have id 0, sometimes 1. This enables both possibilities.

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
        
    inner = inner[:-2] # remove final ",\n"

    out = em.expand("""tags:
  [
@(str)
  ]
bundles:
  [
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