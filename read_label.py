#Program that search in each annotators file, after good area. Then it count the midpoint in the bounding box 
with open("test_image2.txt", "r") as f:  #The annotators file for the give image 
	for line in f:
		words = line.split()
		if words:
			if words[0]=="stone":
				x1=float(words[4])
				y1=float(words[5])
				x2=float(words[6])
				y2=float(words[7])

				y_midle=(y2-y1)/2 + y1
				x_midle=(x2-x1)/2 + x1
				print("y_midle", y_midle)
				print("x_midle", x_midle)	
				
