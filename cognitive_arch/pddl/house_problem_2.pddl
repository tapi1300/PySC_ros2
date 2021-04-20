(define (problem cognitive_problem)
(:domain cognitive_domain)
(:objects 
    salon cocina h1 h2 b1 b2 - room
    pasillo - corridor
    o1 - object
    r - robot
    z1 z2 z3 z4 z5 - zone
)
(:init 

    (robot_at r salon)
    (robot_available r)
    (not_robot_at_zone r)
    (free r)
    (object_at o1 h2)
    (connected salon pasillo)
    (connected pasillo salon)
    (connected h1 pasillo)
    (connected pasillo h1)
    (connected h2 pasillo)
    (connected pasillo h2)
    (connected b1 pasillo)
    (connected pasillo b1)
    (connected b2 pasillo)
    (connected pasillo b2)
    (connected salon cocina)
    (connected cocina salon)
    (zone_at z1 salon)
    (zone_at z2 salon)
    (zone_at z3 cocina)
    (zone_at z4 cocina)
    (zone_at z5 h1)
)

(:goal 
    (and
        (object_at o1 z1)
        (robot_at r h1)
    )
)
)