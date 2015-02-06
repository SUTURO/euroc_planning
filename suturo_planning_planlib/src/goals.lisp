(in-package :planlib)

(def-goal (achieve (map-scanned))
  (scan-map-mast-cam)
  (scan-shadow))

(def-goal (achieve (objects-informed))
  ; TODO: Implement me correctly
  (achieve `(unknown-scanned))
  (achieve `(object-classified nil))
  (achieve `(pose-estimated nil)))

(def-goal (achieve (unknown-scanned))
  ; TODO: Implement me
  )

(def-goal (achieve (object-classified ?object))
  ; TODO: Implement me
  )

(def-goal (achieve (pose-estimated ?object))
  ; TODO: Implement me
  )

(def-goal (achieve (objects-in-place ?objects))
  ; TODO: Implement me correcty
  (achieve `(object-in-hand nil))
  (achieve `(object-placed-at nil nil)))
