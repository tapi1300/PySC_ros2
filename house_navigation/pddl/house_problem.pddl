(define (problem house_problem)
(:domain house_domain)
(:objects 
    r1 r2 r3 r4  - room
    c1 c2 c3 - corridor
    o1 o2 - object
    r - robot
    z1 z2 z3 z4 z5 - zone
)
(:init 

    (robot_at r r1)
    (object_at o1 r3)
    (connected r2 r3)
    (connected r3 r2)
    (connected r1 c1)
    (connected c1 r1)
    (connected r2 c1)
    (connected c1 r2)
    (robot_available r)
    (not_robot_at_zone r)
    (zone_at z1 r2)
    (free r)
)

(:goal 
    (and
        (object_at o1 z1)
        (robot_at r r1)
    )
)
)