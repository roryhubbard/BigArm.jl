using BigArm
using CoordinateTransformations: Translation
using StaticArrays: SVector

function main()
  stick = RectangularTube(3.0, 3.0, 288.0, 0.120)  # 3×3″ 11-gauge tube, 24′ mill stick
  blank = saw(stick, 103.0)                        # cut to length, inches

  front = datum(blank, SVector(0, -1, 0))          # front wall datum (-Y normal)
  hole  = Circle(1.0)
  upright = blank
  for y in 2.0:2.0:100.0                           # (x, y) from bottom-left corner of front wall
      d = Datum(front.transform ∘ Translation(1.5, y, 0.0))
      upright = drill(upright, hole, d, ThroughAll())
  end

  return visualize(ManufacturingGraph(upright))
end

main()
