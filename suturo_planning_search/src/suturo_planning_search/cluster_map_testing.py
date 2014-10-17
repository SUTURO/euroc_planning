from suturo_planning_search.cluster_map import ClusterRegions
from suturo_planning_search.cell import Cell

__author__ = 'pmania'

def test_field1():
    c = Cell()
    c.set_free()
    field = [[c for x in xrange(15)] for x in xrange(15)]

    field[0][6] = Cell()
    field[0][6].set_unknown()
    field[0][7] = Cell()
    field[0][7].set_unknown()
    field[0][8] = Cell()
    field[0][8].set_unknown()
    field[2][6] = Cell()
    field[2][6].set_unknown()
    field[2][7] = Cell()
    field[2][7].set_unknown()
    field[2][8] = Cell()
    field[2][8].set_unknown()
    return field



cm = ClusterRegions()
cm.set_field(test_field1())
cm.print_field()

print "Returned regions: " + str(cm.group_regions())
print "Grouping done"
cm.print_segmented_field()
c = cm.get_result_map()

regions = cm.get_result_regions()
print "Region count: " + str(len(regions))
for r in regions:
    print r
    print "Distance of region to (0,0): " + str(r.euclidean_distance_to_avg(0,0))
