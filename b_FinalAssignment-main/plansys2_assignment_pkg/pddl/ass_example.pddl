(define (domain robotAruco)
(:requirements :strips :typing :adl :durative-actions)
  (:types 
    marker 
    position
    logic
  )

  ;; Predicates
  (:predicates
    (at_robot ?p0 - position)
    (at_marker ?m - marker ?p1 - position)
    (connected_pos ?pos1 - position ?pos2 - position)
    (reached ?m - marker)
    (order1 ?l - logic)
    (order2 ?l - logic)
  )

  ;; Durative Actions
  (:durative-action go_to_spot
    :parameters (?m - marker ?from ?to - position ?l - logic)
    :duration (= ?duration 10)
    :condition (and
      (at start(at_robot ?from))
      (over all(at_marker ?m ?to))
      (over all(connected_pos ?from ?to))
      (at start(order1 ?l))
    )
    :effect (and
      (at end(at_robot ?to))
      (at end(not (at_robot ?from)))
      (at end(not (order1 ?l)))
      (at end(order2 ?l))
    )
  )

  (:durative-action find_marker
    :parameters (?m - marker ?pos - position ?l - logic)
    :duration (= ?duration 5)
    :condition (and
      (over all(at_robot ?pos))
      (over all(at_marker ?m ?pos))
      (at start(order2 ?l))
    )
    :effect (and
      (at end(reached ?m))
      (at end(order1 ?l))
      (at end(not(order2 ?l)))
    )
  )
)
