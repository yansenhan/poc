import numpy as np

# Create a 21x21 matrix filled with zeros
matrix = np.zeros((21, 21), dtype=int)

# Define relationships based on the descriptions
# Bedroom
# Define relationships based on the descriptions
# Bedroom
matrix[1, 1] = 1  # Bed - Bed
matrix[1, 3] = 1  # Bed - Wardrobe
matrix[1, 5] = 1  # Bed - Chest of Drawers
matrix[1, 14] = 1  # Bed - Stand
matrix[1, 12] = 1  # Bed - Shoe Rack
matrix[1, 4] = 1  # Bed - Chair

matrix[3, 1] = 1  # Wardrobe - Bed
matrix[3, 5] = 1  # Wardrobe - Chest of Drawers
matrix[3, 14] = 1  # Wardrobe - Stand
matrix[3, 12] = 1  # Wardrobe - Shoe Rack
matrix[3, 4] = 1  # Wardrobe - Chair

matrix[5, 1] = 1  # Chest of Drawers - Bed
matrix[5, 3] = 1  # Chest of Drawers - Wardrobe
matrix[5, 14] = 1  # Chest of Drawers - Stand
matrix[5, 12] = 1  # Chest of Drawers - Shoe Rack
matrix[5, 4] = 1  # Chest of Drawers - Chair

matrix[14, 1] = 1  # Stand - Bed
matrix[14, 3] = 1  # Stand - Wardrobe
matrix[14, 5] = 1  # Stand - Chest of Drawers
matrix[14, 12] = 1  # Stand - Shoe Rack
matrix[14, 4] = 1  # Stand - Chair

matrix[12, 1] = 1  # Shoe Rack - Bed
matrix[12, 3] = 1  # Shoe Rack - Wardrobe
matrix[12, 5] = 1  # Shoe Rack - Chest of Drawers
matrix[12, 14] = 1  # Shoe Rack - Stand
matrix[12, 4] = 1  # Shoe Rack - Chair

matrix[4, 1] = 1  # Chair - Bed
matrix[4, 3] = 1  # Chair - Wardrobe
matrix[4, 5] = 1  # Chair - Chest of Drawers
matrix[4, 14] = 1  # Chair - Stand
matrix[4, 12] = 1  # Chair - Shoe Rack

# Bathroom
matrix[0, 0] = 1  # Bathtub - Bathtub
matrix[0, 13] = 1  # Bathtub - Sink
matrix[0, 17] = 1  # Bathtub - Toilet
matrix[0, 3] = 1  # Bathtub - Cabinet
matrix[0, 4] = 1  # Bathtub - Chair
matrix[0, 11] = 1  # Bathtub - Shelves

matrix[13, 0] = 1  # Sink - Bathtub
matrix[13, 13] = 1  # Sink - Sink
matrix[13, 17] = 1  # Sink - Toilet
matrix[13, 3] = 1  # Sink - Cabinet
matrix[13, 4] = 1  # Sink - Chair
matrix[13, 11] = 1  # Sink - Shelves

matrix[17, 0] = 1  # Toilet - Bathtub
matrix[17, 13] = 1  # Toilet - Sink
matrix[17, 17] = 1  # Toilet - Toilet
matrix[17, 3] = 1  # Toilet - Cabinet
matrix[17, 4] = 1  # Toilet - Chair
matrix[17, 11] = 1  # Toilet - Shelves

matrix[3, 0] = 1  # Cabinet - Bathtub
matrix[3, 13] = 1  # Cabinet - Sink
matrix[3, 17] = 1  # Cabinet - Toilet
matrix[3, 3] = 1  # Cabinet - Cabinet
matrix[3, 4] = 1  # Cabinet - Chair
matrix[3, 11] = 1  # Cabinet - Shelves

matrix[4, 0] = 1  # Chair - Bathtub
matrix[4, 13] = 1  # Chair - Sink
matrix[4, 17] = 1  # Chair - Toilet
matrix[4, 3] = 1  # Chair - Cabinet
matrix[4, 4] = 1  # Chair - Chair
matrix[4, 11] = 1  # Chair - Shelves

matrix[11, 0] = 1  # Shelves - Bathtub
matrix[11, 13] = 1  # Shelves - Sink
matrix[11, 17] = 1  # Shelves - Toilet
matrix[11, 3] = 1  # Shelves - Cabinet
matrix[11, 4] = 1  # Shelves - Chair
matrix[11, 11] = 1  # Shelves - Shelves

# Living Room
matrix[6, 6] = 1  # Couch - Couch
matrix[6, 4] = 1  # Couch - Chair
matrix[6, 16] = 1  # Couch - Table
matrix[6, 3] = 1  # Couch - Cabinet
matrix[6, 11] = 1  # Couch - Shelves

matrix[4, 6] = 1  # Chair - Couch
matrix[4, 4] = 1  # Chair - Chair
matrix[4, 16] = 1  # Chair - Table
matrix[4, 3] = 1  # Chair - Cabinet
matrix[4, 11] = 1  # Chair - Shelves

matrix[16, 6] = 1  # Table - Couch
matrix[16, 4] = 1  # Table - Chair
matrix[16, 16] = 1  # Table - Table
matrix[16, 3] = 1  # Table - Cabinet
matrix[16, 11] = 1  # Table - Shelves

matrix[3, 6] = 1  # Cabinet - Couch
matrix[3, 4] = 1  # Cabinet - Chair
matrix[3, 16] = 1  # Cabinet - Table
matrix[3, 3] = 1  # Cabinet - Cabinet
matrix[3, 11] = 1  # Cabinet - Shelves

matrix[11, 6] = 1  # Shelves - Couch
matrix[11, 4] = 1  # Shelves - Chair
matrix[11, 16] = 1  # Shelves - Table
matrix[11, 3] = 1  # Shelves - Cabinet
matrix[11, 11] = 1  # Shelves - Shelves

# Kitchen
matrix[7, 7] = 1  # Counter - Counter
matrix[7, 4] = 1  # Counter - Chair
matrix[7, 3] = 1  # Counter - Cabinet
matrix[7, 10] = 1  # Counter - Serving Cart
matrix[7, 11] = 1  # Counter - Shelves

matrix[4, 7] = 1  # Chair - Counter
matrix[4, 4] = 1  # Chair - Chair
matrix[4, 3] = 1  # Chair - Cabinet
matrix[4, 10] = 1  # Chair - Serving Cart
matrix[4, 11] = 1  # Chair - Shelves

matrix[3, 7] = 1  # Cabinet - Counter
matrix[3, 4] = 1  # Cabinet - Chair
matrix[3, 10] = 1  # Cabinet - Serving Cart
matrix[3, 3] = 1  # Cabinet - Cabinet
matrix[3, 11] = 1  # Cabinet - Shelves

matrix[10, 7] = 1  # Serving Cart - Counter
matrix[10, 4] = 1  # Serving Cart - Chair
matrix[10, 3] = 1  # Serving Cart - Cabinet
matrix[10, 10] = 1  # Serving Cart - Serving Cart
matrix[10, 11] = 1  # Serving Cart - Shelves

matrix[11, 7] = 1  # Shelves - Counter
matrix[11, 4] = 1  # Shelves - Chair
matrix[11, 3] = 1  # Shelves - Cabinet
matrix[11, 10] = 1  # Shelves - Serving Cart
matrix[11, 11] = 1  # Shelves - Shelves

# Laundry Room
matrix[20, 20] = 1  # Washer Dryer - Washer Dryer
matrix[20, 3] = 1  # Washer Dryer - Cabinet
matrix[20, 11] = 1  # Washer Dryer - Shelves

matrix[3, 20] = 1  # Cabinet - Washer Dryer
matrix[3, 3] = 1  # Cabinet - Cabinet
matrix[3, 11] = 1  # Cabinet - Shelves

matrix[11, 20] = 1  # Shelves - Washer Dryer
matrix[11, 3] = 1  # Shelves - Cabinet
matrix[11, 11] = 1  # Shelves - Shelves

# Office/Study
matrix[16, 16] = 1  # Table - Table
matrix[16, 4] = 1  # Table - Chair
matrix[16, 8] = 1  # Table - Filing Cabinet
matrix[16, 3] = 1  # Table - Cabinet
matrix[16, 11] = 1  # Table - Shelves

matrix[4, 16] = 1  # Chair - Table
matrix[4, 4] = 1  # Chair - Chair
matrix[4, 8] = 1  # Chair - Filing Cabinet
matrix[4, 3] = 1  # Chair - Cabinet
matrix[4, 11] = 1  # Chair - Shelves

matrix[8, 16] = 1  # Filing Cabinet - Table
matrix[8, 4] = 1  # Filing Cabinet - Chair
matrix[8, 8] = 1  # Filing Cabinet - Filing Cabinet
matrix[8, 3] = 1  # Filing Cabinet - Cabinet
matrix[8, 11] = 1  # Filing Cabinet - Shelves

matrix[3, 16] = 1  # Cabinet - Table
matrix[3, 4] = 1  # Cabinet - Chair
matrix[3, 8] = 1  # Cabinet - Filing Cabinet
matrix[3, 3] = 1  # Cabinet - Cabinet
matrix[3, 11] = 1  # Cabinet - Shelves

matrix[11, 16] = 1  # Shelves - Table
matrix[11, 4] = 1  # Shelves - Chair
matrix[11, 8] = 1  # Shelves - Filing Cabinet
matrix[11, 3] = 1  # Shelves - Cabinet
matrix[11, 11] = 1  # Shelves - Shelves

# 卧室包括:床 (bed)衣柜 (wardrobe)衣柜 (chest_of_drawers)梳妆台 (stand)鞋架 (shoe_rack)椅子 (chair),
#浴室包括浴缸 (bathtub)水槽 (sink)厕所 (toilet)柜子 (cabinet)椅子 (chair)架子 (shelves)，
#客厅包括沙发 (couch)椅子 (chair)茶几 (table)电视柜 (cabinet)架子 (shelves)书架 (shelves)，
#厨房包括柜台 (counter)椅子 (chair)贮藏柜 (cabinet)餐车 (serving_cart)架子 (shelves)
#洗衣间包括洗衣机/烘干机 (washer_dryer)柜子 (cabinet)架子 (shelves)，
#办公室/书房包括书桌 (table)椅子 (chair)文件柜 (filing_cabinet)架子 (shelves)柜子 (cabinet) 
# Save the matrix to a text file
np.savetxt('/home/ubuntu/zyp/home-robot/src/home_robot/home_robot/navigation_policy/object_navigation/furniture_relationships.txt', matrix, fmt='%d')
print(matrix)