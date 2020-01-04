import os
home_path=os.getcwd() 
img_list = os.listdir(home_path)
input_name = input()
for i in range(0,len(img_list)):

  filename = str(input_name)+"_"+img_list[i]
  print(filename)
  os.rename(img_list[i],filename)
