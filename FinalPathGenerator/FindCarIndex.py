def findcarindex(car_initial_x,car_initial_y,data):
    for row in data:
        up_point = float(row[7])
        down_point = float(row[6])
        right_point = float(row[8])
        left_point = float(row[9])
        if car_initial_x<=right_point and car_initial_x>= left_point and car_initial_y<= up_point and car_initial_y>= down_point:
            # print("****")
            car_initial_index_x=int(row[0])
            car_initial_index_y=int(row[1])
            # print(row[0],row[1])
            return car_initial_index_x,car_initial_index_y


        # else:
        #     if direction =="K" or direction =="G":
        #         if car_initial_x<=right_point+1.5 and car_initial_x>= left_point-1.5 and car_initial_y<= up_point and car_initial_y>= down_point:
        #             print("****")
        #             car_initial_index_x=int(row[0])
        #             car_initial_index_y=int(row[1])
        #             print(row[0],row[1])
        #             return car_initial_index_x,car_initial_index_y
        #     elif  direction =="D" or direction =="B":
        #         if car_initial_x<=right_point and car_initial_x>= left_point and car_initial_y<= up_point+1.5 and car_initial_y>= down_point-1.5:
        #             print("****")
        #             car_initial_index_x=int(row[0])
        #             car_initial_index_y=int(row[1])
        #             print(row[0],row[1])
        #             return car_initial_index_x,car_initial_index_y

