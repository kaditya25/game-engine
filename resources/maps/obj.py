import json


# Bottom 4 points are (x1,y1,z1) through (x4,y4,z4)
# Top 4 points are (x5,y5,z5) through (x8,y8,z8)
# Make sure (x1,y1,z1) & (x4,y4,z4) on the same side (x5,y5,z5) & (x8,y8,z8)
# etc

def printPrism(obj):
    print("[")
    l1 = len(obj) - 1
    for i, ar in enumerate(obj):
        print("  [")
        l2 = len(ar) - 1
        for j, it in enumerate(ar):
            print("    ", end = '')
            print(it, end = '')
            if j <  l2:
                print(",")

        if i <  l1:
            print("\n  ],")
        else:
            print("\n  ]")
    print("],")

def prism(p1,p2,p3,p4,p5,p6,p7,p8):
    # Assumes bottom p1->p2->p3->p4->p1
    # Assumes top p5->p8->p7->p6->p5
    # Assumes side1 p4->p3->p7->p8->p4
    # Assumes side2 p1->p5->p6->p2->p1
    # Assumes side3 p1->p4->p8->p5->p1
    # Assumes side4 p2->p6->p7->p3->p2

    x1,y1,z1 = p1[0],p1[1],p1[2]
    x2,y2,z2 = p2[0],p2[1],p2[2]
    x3,y3,z3 = p3[0],p3[1],p3[2]
    x4,y4,z4 = p4[0],p4[1],p4[2]
    x5,y5,z5 = p5[0],p5[1],p5[2]
    x6,y6,z6 = p6[0],p6[1],p6[2]
    x7,y7,z7 = p7[0],p7[1],p7[2]
    x8,y8,z8 = p8[0],p8[1],p8[2]
    obj = [
        # Bottom p1->p2->p3->p4->p1
        [
            [x1,y1,z1,x2,y2,z2],
            [x2,y2,z2,x3,y3,z3],
            [x3,y3,z3,x4,y4,z4],
            [x4,y4,z4,x1,y1,z1]
        ],
        # Top p5->p8->p7->p6->p5
        [
            [x5,y5,z5,x8,y8,z8],
            [x8,y8,z8,x7,y7,z7],
            [x7,y7,z7,x6,y6,z6],
            [x6,y6,z6,x5,y5,z5]
        ],
        # Side with p4->p3->p7->p8->p4
        [
            [x4,y4,z4,x3,y3,z3],
            [x3,y3,z3,x7,y7,z7],
            [x7,y7,z7,x8,y8,z8],
            [x8,y8,z8,x4,y4,z4]
        ],
        # Side with p1->p5->p6->p2->p1
        [
            [x1,y1,z1,x5,y5,z5],
            [x5,y5,z5,x6,y6,z6],
            [x6,y6,z6,x2,y2,z2],
            [x2,y2,z2,x1,y1,z1]
        ],
        # Side with p1->p4->p8->p5->p1
        [
            [x1,y1,z1,x4,y4,z4],
            [x4,y4,z4,x8,y8,z8],
            [x8,y8,z8,x5,y5,z5],
            [x5,y5,z5,x1,y1,z1]
        ],
        # Side with p2->p6->p7->p3->p2
        [
            [x2,y2,z2,x6,y6,z6],
            [x6,y6,z6,x7,y7,z7],
            [x7,y7,z7,x3,y3,z3],
            [x3,y3,z3,x2,y2,z2]
        ]
    ]
    printPrism(obj)


# 1 # Points in order that can be connected
p1 = [-20, 13.7, -5.15]
p2 = [-19.8, 13.7, -5.15]
p3 = [-19.8, 14.85, -5.15]
p4 = [-20, 14.85, -5.15]
p5 = [-20, 13.7, -0.882]
p6 = [-19.8, 13.7, -0.882]
p7 = [-19.8, 14.85, -0.882]
p8 = [-20, 14.85, -0.882]
prism(p1,p2,p3,p4,p5,p6,p7,p8)
# 3
p1 = [-20, 14.85, -5.15]
p2 = [-19.8, 14.85, -5.15]
p3 = [-19.8, 15.85, -5.15]
p4 = [-20, 15.85, -5.15]
p5 = [-20, 14.85, -3.516]
p6 = [-19.8, 14.85, -3.516]
p7 = [-19.8, 15.85, -3.516]
p8 = [-20, 15.85, -3.516]
prism(p1,p2,p3,p4,p5,p6,p7,p8)
# 2
p1 = [-20, 14.85, -2.516]
p2 = [-19.8, 14.85, -2.516]
p3 = [-19.8, 15.85, -2.516]
p4 = [-20, 15.85, -2.516]
p5 = [-20, 14.85, -0.882]
p6 = [-19.8, 14.85, -0.882]
p7 = [-19.8, 15.85, -0.882]
p8 = [-20, 15.85, -0.882]
prism(p1,p2,p3,p4,p5,p6,p7,p8)
# 4
p1 = [-20, 15.85, -5.15]
p2 = [-19.8, 15.85, -5.15]
p3 = [-19.8, 17, -5.15]
p4 = [-20, 17, -5.15]
p5 = [-20, 15.85, -0.882]
p6 = [-19.8, 15.85, -0.882]
p7 = [-19.8, 17, -0.882]
p8 = [-20, 17, -0.882]
prism(p1,p2,p3,p4,p5,p6,p7,p8)

