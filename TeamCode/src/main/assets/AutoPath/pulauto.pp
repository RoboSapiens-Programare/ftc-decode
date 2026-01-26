{
  "startPoint": {
    "x": 56,
    "y": 8,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-19wg3eluorn",
      "name": "",
      "endPoint": {
        "x": 58,
        "y": 22,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 100
      },
      "controlPoints": [
        {
          "x": 65.99916787705264,
          "y": 20.155378116453573
        }
      ],
      "color": "#878D8B",
      "eventMarkers": [],
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mks49iwx-ddf02j",
      "name": "",
      "endPoint": {
        "x": 22,
        "y": 22,
        "heading": "linear",
        "reverse": false,
        "startDeg": 100,
        "endDeg": 90
      },
      "controlPoints": [],
      "color": "#9D897A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 118,
          "y": 144
        },
        {
          "x": 138,
          "y": 118
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#fca5a5"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 7,
          "y": 118
        },
        {
          "x": 26,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#0b08d9",
      "fillColor": "#fca5a5"
    }
  ],
  "sequence": [
    {
      "kind": "wait",
      "id": "mks48ass-y1lcx7",
      "name": "",
      "durationMs": 20000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "line-19wg3eluorn"
    },
    {
      "kind": "path",
      "lineId": "mks49iwx-ddf02j"
    }
  ],
  "version": "1.2.1",
  "timestamp": "2026-01-24T09:39:09.756Z"
}