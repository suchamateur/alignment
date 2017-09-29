#coding=utf-8
import xlrd
import numpy as np
import cv2
import time
import Image, ImageDraw, ImageFont
import sys
import ConfigParser, winsound

day_str_clock = 7
day_end_clock = 24

def get_elapsed_time(txt, str_hour):
    strs = txt.split(':')
    t = [0, 0, 0]
    for i in range(0, len(strs)):
        if i > 2:
            break
        t[i] = float(strs[len(strs)-1-i])
    t[2] = t[2] - str_hour
    return t[2]*3600 + t[1]*60 + t[0]

def is_day_one(txt):
    strs = txt.split(':')
    if len(strs) > 3:
        return False
    else:
        return True

def trim2sec(txt):
    strs = txt.split('.')
    return strs[0]

def time2str(t, str_hour):
    hour = str_hour + t // 3600
    minute = (t % 3600) // 60
    second = int(t % 60)
    return '%.2d:%.2d:%.2d'%(hour, minute, second)

def get_column_box_list(strs):
    boxes = [[],[],[],[]]
    for i in range(0, 4):
        if len(strs[i]) > 0:
            box_strs = strs[i].split(';')
            if len(box_strs) > 0:
                for j in range(0, len(box_strs)):
                    box_code = box_strs[j].split('(')
                    boxes[i].append(box_code[0])
    return boxes

def get_operation_type(op_str):
    type_inbound = '入库'
    type_outbound = '出库'
    type_kitting = '配料'
    type_inbound = type_inbound.decode('utf8')
    type_outbound = type_outbound.decode('utf8')
    type_kitting = type_kitting.decode('utf8')
    
    if unicode(op_str) == type_outbound:
        return 1
    elif unicode(op_str) == type_inbound:
        return 2
    elif unicode(op_str) == type_kitting:
        return 3
    else:
        return 0
    
def array_to_string(arr):
    out_str = ''
    for i in range(0, len(arr)):
        out_str = out_str + arr[i] +';'
    return out_str

class object_agv:
    def __init__(self, row):
        if row[0] == 'TB':
            self.type = 1
        else:
            self.type = 2
        self.no = int(row[2])
        self.depart_time = get_elapsed_time(row[3], day_str_clock)
        self.return_time = get_elapsed_time(row[4], day_str_clock)
        self.cargos = row[5].split(';')
        if len(self.cargos) > 0:
            if self.cargos[len(self.cargos)-1] == '':
                self.cargos.pop()
        self.column_cargos = [row[6].split(';'), row[7].split(';'), row[8].split(';'), row[9].split(';')]
        for i in range(0, len(self.column_cargos)):
            if len(self.column_cargos[i]) > 0:
                j = len(self.column_cargos[i]) - 1
                if self.column_cargos[i][j] == '':
                    self.column_cargos[i].pop()
        self.is_day_one = is_day_one(row[3])
        
class object_operation:
    def __init__(self, row):
        self.label = row[0]
        self.lane = int(row[1])
        self.str_time = get_elapsed_time(row[4], day_str_clock)
        self.end_time = get_elapsed_time(row[5], day_str_clock)
        self.is_day_one = is_day_one(row[4])
        self.type = get_operation_type(row[3])
        if self.type == 1:
            # outbound
            self.work = row[7].split(';')
            if len(self.work) > 0:
                if self.work[len(self.work) - 1] == '':
                    self.work.pop()
            sub_strs = row[8].split('-')
            self.agv_type = int(sub_strs[0])
            self.agv_no = int(sub_strs[1])
            self.op = int(row[2])
        elif self.type == 3:
            # kitting
            self.work = row[7].split(';')
            if len(self.work) > 0:
                if self.work[len(self.work) - 1] == '':
                    self.work.pop()

class gantt_drawer:
    def __init__(self, agv_list, op_list):
        # load excel file
        self.agvs = agv_list
        self.ops = op_list
        
        # image parameters
        self.ox = 50
        self.step = 40
        self.oy = self.step / 2
        self.width = 16000
        self.first_col_width = 90
        self.bar_height = 30    
        
        self.gantt_img = np.zeros((21 * self.step + 2* self.oy, self.width + 2* self.ox + self.first_col_width, 3), np.uint8) + 255
        
    def draw_table(self, str_row, nrow, txts):
        for i in range(0, nrow + 1):
            #horizontal lines
            cv2.line(self.gantt_img, (self.ox, self.oy + self.step * (str_row + i)), (self.ox + self.width + self.first_col_width, self.oy + self.step * (str_row + i)), (0,0,0), 1, cv2.LINE_AA)
            if i > 1 and i < nrow:
                #horizontal gray lines
                cv2.line(self.gantt_img, (self.ox + self.first_col_width, self.oy + self.bar_height+ self.step * (str_row + i)), (self.ox + self.width + self.first_col_width, self.oy + self.bar_height + self.step * (str_row + i)), (210,210,210), 1, cv2.LINE_AA)
        
        #vertical lines
        cv2.line(self.gantt_img, (self.ox + self.first_col_width + self.width, self.oy + self.step * str_row), (self.ox + self.first_col_width + self.width, self.oy + self.step * (str_row + nrow)), (0,0,0), 1, cv2.LINE_AA)
        cv2.line(self.gantt_img, (self.ox, self.oy + self.step * str_row), (self.ox, self.oy + self.step * (str_row + nrow)), (0,0,0), 1, cv2.LINE_AA)
        
        # time stamps
        time_stamps = []
        font = cv2.FONT_HERSHEY_SIMPLEX
        for i in range(0, 24):
            if i >= day_str_clock and i <= day_end_clock:
                time_stamps.append(i)
        self.scale = self.width / 3600.0 / len(time_stamps)
        for i in range(0, len(time_stamps)):
            cv2.putText(self.gantt_img, '%d:00'%time_stamps[i], (int(3600 * i * self.scale) + self.first_col_width + self.ox - 15, self.oy + (2 + str_row) * self.step - 14), font, 0.6, (0,0,0), 1, cv2.LINE_AA)
            cv2.line(self.gantt_img, (int(3600 * i * self.scale) + self.first_col_width + self.ox, self.oy + (2 + str_row) * self.step - 10), (int(3600 * i * self.scale) + self.first_col_width + self.ox, self.oy + (nrow + str_row) * self.step),(0,0,0))
            # gray time stamps
            for j in range(1, 6):
                cv2.putText(self.gantt_img, '%d:%d0'%(time_stamps[i],j), (int(3600*i*self.scale+600*self.scale*j)+self.first_col_width+self.ox-13, self.oy+(2+str_row)*self.step-14), font, 0.5, (0,0,0), 1, cv2.LINE_AA)
                cv2.line(self.gantt_img, (int(3600*i*self.scale+600*self.scale*j)+self.first_col_width+self.ox, self.oy+(2+str_row)*self.step-10), (int(3600*i*self.scale+600*self.scale*j)+self.first_col_width+self.ox, self.oy+(nrow+str_row)*self.step),(210,210,210))
        
        for i in range(0, len(txts)):
            if i == 0:
                cv2.putText(self.gantt_img, txts[i], (self.ox + 10, self.oy + (str_row + 1) * self.step - 10), font, 0.7, (0,0,0), 1, cv2.LINE_AA)
            else:
                cv2.putText(self.gantt_img, txts[i], (self.ox + 10, self.oy + (str_row + 2 + i) * self.step -13), font, 0.5, (0,0,0), 1, cv2.LINE_AA)  
    
    def draw_bar(self, lx, ly, bw, bh, color, txt):
        font = cv2.FONT_HERSHEY_SIMPLEX
        box = []
        box.append((lx, ly))
        box.append((lx, ly + bh))
        box.append((lx + bw, ly + bh))
        box.append((lx + bw, ly))
        
        cv2.fillConvexPoly(self.gantt_img, np.int32(box), (0,0,0))
        cv2.fillPoly(self.gantt_img, [np.int32(box)], color)
        cv2.putText(self.gantt_img, txt, (int(lx) + 10, int(ly) + 20), font, 0.5, (255,255,255), 1, cv2.LINE_AA)
        cv2.rectangle(self.gantt_img, (int(lx), int(ly)), (int(lx + bw), int(ly + bh)), (0,0,0))
                
    def draw_gantt(self):
        #draw table
        row_strs = [0, 5, 10, 16]
        
        self.draw_table(row_strs[0], 4, ['AGV Day 1', 'TB', 'DTA'])
        self.draw_table(row_strs[1], 4, ['AGV Day 2', 'TB', 'DTA'])
        self.draw_table(row_strs[2], 5, ['Kit Station Day 1', 'Station 1', 'Station 2', 'Station 3'])
        self.draw_table(row_strs[3], 5, ['Kit Station Day 2', 'Station 1', 'Station 2', 'Station 3'])
        
        #draw agv schedule
        for a in self.agvs:          
            box_left_x = self.ox + self.first_col_width + a.depart_time * self.scale
            box_left_y = self.oy + 2 * self.step
            if a.type == 2:
                box_left_y += self.step
            if not a.is_day_one:
                box_left_y += self.step * row_strs[1]
            
            txt = '%d'%(a.no)
            self.draw_bar(box_left_x, box_left_y, (a.return_time - a.depart_time) * self.scale, self.bar_height, (102,0,102), txt)
        
        #draw warehouse schedule    
        for op in ops:
            box_left_x = self.ox + self.first_col_width + op.str_time * self.scale
            if op.is_day_one:
                box_left_y = self.oy + (op.lane + 1 + row_strs[2]) * self.step
            else:
                box_left_y = self.oy + (op.lane + 1 + row_strs[3]) * self.step
            
            color = [0,0,0]
            if op.type == 2:
                #inbound
                color = [80,176,0]
            elif op.type == 1:
                #outbound
                color = [255,102,51]
            elif op.type == 3:
                #kitting
                color = [0,51,204]
            self.draw_bar(box_left_x, box_left_y, (op.end_time - op.str_time) * self.scale, self.bar_height, color, op.label) 
 
    def get_image(self):
        return self.gantt_img

if __name__ == '__main__':
    
#     vw = cv2.VideoWriter('c:\\tmp\\clip.avi', cv2.VideoWriter_fourcc('C','V','I','D'), 25, (1366, 768))
#     for i in range(300, 450):
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#         frame = cv2.imread('c:\\tmp\\clip\\frame%.5d.jpg'%i)
#         cv2.imshow('frame', frame)
#         vw.write(frame)
#     vw.release()
#     print'done'
#     exit()
    
    #load excel
    xls_file = xlrd.open_workbook('src\\gantt.xlsx')
    tbl_agv = xls_file.sheets()[0]
    tbl_gantt = xls_file.sheets()[1]
    agvs = []
    ops = []
    for r in range(1, tbl_agv.nrows):
        row = tbl_agv.row_values(r)
        agvs.append(object_agv(row))
    for r in range(1, tbl_gantt.nrows):
        row = tbl_gantt.row_values(r)
        ops.append(object_operation(row))
        
    #generate gantt image
    gdrawer = gantt_drawer(agvs, ops)
    gdrawer.draw_gantt()
    whole_gantt_img = gdrawer.get_image()    

    config = ConfigParser.SafeConfigParser()
    config.read('src\config.ini')
    if config.getint('Gantt','save_to_disk') == 1:
        cv2.imwrite('src\\gantt.jpg', whole_gantt_img)
        print 'image saved to src folder'
    #exit()
    speed = config.getint('Display', 'speed')    
    day = config.getint('Display', 'day')
    # filter by day
    day_agvs = []
    day_ops = []
    is_day_one_schedule = True;
    if day == 2:
        is_day_one_schedule = False;
    for ag in agvs:
        if ag.is_day_one == is_day_one_schedule:
            day_agvs.append(ag)
    for op in ops:
        if op.is_day_one == is_day_one_schedule:
            day_ops.append(op)    
    
    img_width = config.getint('Display', 'image_width')
    img_height = config.getint('Display', 'image_height')
    img_scale = 2
    spacing = 60    
    count = 1
    seconds = (day_end_clock - day_str_clock) * 3600
    
    xls_file = xlrd.open_workbook('src\\gantt.xlsx')
    tbl_agv = xls_file.sheets()[0]
    tbl_gantt = xls_file.sheets()[1]
    row = tbl_gantt.row_values(1)
    gantt_str_time = get_elapsed_time(row[4], 7)
    gantt_str_time_changed = False
    for i in range(2, tbl_gantt.nrows):
        row = tbl_gantt.row_values(i)
        if gantt_str_time == get_elapsed_time(row[4], 7):
            if gantt_str_time_changed:
                if day == 1:
                    gantt_str_row = 1
                    gantt_end_row = i
                else:
                    gantt_str_row = i
                    gantt_end_row = tbl_gantt.nrows
                break;
        else:
            gantt_str_time_changed = True
    
#     print gantt_str_row, ', ', gantt_end_row
    
    #img_gantt = cv2.imread('src\\gantt.jpg')    
    whole_gantt_img = cv2.cvtColor(whole_gantt_img, cv2.COLOR_BGR2RGB)
    img_shape = whole_gantt_img.shape
    whole_gantt_img = cv2.resize(whole_gantt_img, (img_shape[1]/img_scale, img_shape[0]/img_scale))
    img_shape = whole_gantt_img.shape
    img = np.zeros((img_height, img_width, 3), np.uint8) + 255
    sec_to_px = 16000.0 / seconds / img_scale
    #start y coordinate for each table
    tys = [0, 200/img_scale, 400/img_scale, 640/img_scale]
    
    str_time = time.time()
    if speed == 1:
        #start from 7:0:0
        str_time = time.time()
        local_time = time.localtime(str_time)
        t = (local_time.tm_year, local_time.tm_mon, local_time.tm_mday, 7, 0, 0, local_time.tm_wday, local_time.tm_yday, local_time.tm_isdst)
        str_time = time.mktime(t)

    pre_time = str_time
    offset_px = 140 / img_scale
    
    #font = cv2.FONT_HERSHEY_SIMPLEX
    font = ImageFont.truetype('msyh.ttf', 15, encoding='utf-8')
    
    ox = 50
    oy = img_height / 2
    toy = 20 # table offset form the top
    col = int((img_width - ox)/ 5)
    row_height = 20
    
    #decide whether need play sound
    cur_jobs = [0, 0, 0, 0, 0]
    
    #index for ongoing operation in warehouse
    now_stat = [-1, -1, -1]
    while 1:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        cur_time = time.time()
        if (cur_time - str_time) * speed > seconds:
            break
         
        if (cur_time - pre_time) * speed > 1 :            
            #update image
            pass_time = int((cur_time - str_time)*speed)
            time_str = '%.2d:%.2d:%.2d'%(day_str_clock + pass_time // 3600, (pass_time % 3600) // 60, pass_time % 60)
            px = int(pass_time * sec_to_px + offset_px)
            
            img[:toy, :] = 255
            
            if px < img_width / 2:
                # beginn, background image frozen
                if day == 1:                
                    img[toy : toy + tys[1], :] = whole_gantt_img[: tys[1], : img_width]
                    img[toy + tys[1] : toy + tys[1] + (tys[3] - tys[2]), :] = whole_gantt_img[tys[2] : tys[3], : img_width]
                else:
                    img[toy : toy + (tys[2] - tys[1]), :] = whole_gantt_img[tys[1] : tys[2], : img_width]
                    img[toy + (tys[2] - tys[1]) : (toy + (tys[2] - tys[1]) + img_shape[0] - tys[3]), :] = whole_gantt_img[tys[3] :, : img_width]
            elif img_shape[1] - px < img_width / 2:
                # end, background image frozen
                # beginn, background image frozen
                if day == 1:                
                    img[toy : toy + tys[1], :] = whole_gantt_img[: tys[1], (img_shape[1]-img_width):]
                    img[toy + tys[1] : toy + tys[1] + (tys[3] - tys[2]), :] = whole_gantt_img[tys[2] : tys[3], (img_shape[1]-img_width):]
                else:
                    img[toy : toy + (tys[2] - tys[1]), :] = whole_gantt_img[tys[1] : tys[2], (img_shape[1]-img_width):]
                    img[toy + (tys[2] - tys[1]) : (toy + (tys[2] - tys[1]) + img_shape[0] - tys[3]), :] = whole_gantt_img[tys[3] :, (img_shape[1]-img_width):]
                px = img_width - (img_shape[1] - px)
            else:
                if day == 1:                
                    img[toy : toy + tys[1], :] = whole_gantt_img[: tys[1], (px - img_width/2):(px + img_width/2)]
                    img[toy + tys[1] : toy + tys[1] + (tys[3] - tys[2]), :] = whole_gantt_img[tys[2] : tys[3], (px - img_width/2):(px + img_width/2)]
                else:
                    img[toy : toy + (tys[2] - tys[1]), :] = whole_gantt_img[tys[1] : tys[2], (px - img_width/2):(px + img_width/2)]
                    img[toy + (tys[2] - tys[1]) : (toy + (tys[2] - tys[1]) + img_shape[0] - tys[3]), :] = whole_gantt_img[tys[3] :, (px - img_width/2):(px + img_width/2)]
                px = img_width / 2
            if day == 1:
                gantt_height = tys[1] + tys[3] - tys[2] + toy
            else:
                gantt_height = tys[2] - tys[1] + img_shape[0] - tys[3] + toy
            #img = np.resize(img, (img_height, img_width, 3))
            img[gantt_height:img_height, :] = 255
            pil_img = Image.fromarray(img)
            pil_draw = ImageDraw.Draw(pil_img)

            pil_draw.line((px, 50/img_scale, px, gantt_height), (0,0,255), 2)
            pil_draw.text((650, 5), time_str, (0,0,0), font)
            oy = gantt_height + 50            
         
            # draw static text
            for i in range(4):
                if i == 0:
                    pil_draw.text((ox + col * i, oy), 'AGV', (0,0,0), font)
                else:
                    tmp_str = '巷道  %d'%i
                    tmp_str = tmp_str.decode('utf8')
                    pil_draw.text((ox + col * (i+1), oy), tmp_str, (0,0,0), font)
                   
            # update AGV content
            tab_rows = [2, 4, 13, 15]
            pil_draw.text((ox, oy + row_height), 'TB', (0,0,0), font)
            pil_draw.text((ox, oy + row_height*(tab_rows[2]-1)), 'DTA', (0,0,0), font)
            tmp_str = '发车'
            tmp_str = tmp_str.decode('utf8')
            pil_draw.text((ox, oy + row_height*tab_rows[0]), tmp_str, (0,176,240), font)
            pil_draw.text((ox, oy + row_height*tab_rows[2]), tmp_str, (0,176,240), font)
            tmp_str = '装车'
            tmp_str = tmp_str.decode('utf8')
            pil_draw.text((ox, oy + row_height*tab_rows[1]), tmp_str, (255,51,204), font)   
            pil_draw.text((ox, oy + row_height*tab_rows[3]), tmp_str, (255,51,204), font)          

            #update index for current and next agvs
            now_tb = -1
            now_dta = -1
            next_tb = -1
            next_dta = -1
            min_time_diff_tb = 0
            min_time_diff_dta = 0
            for a in range(0, len(day_agvs)):
                cur_agv = day_agvs[a]
                if cur_agv.type == 1:
                    #TB
                    if cur_agv.depart_time <= pass_time and cur_agv.return_time > pass_time:
                        now_tb = a
                    elif cur_agv.depart_time > pass_time:
                        if next_tb < 0:
                            min_time_diff_tb = cur_agv.depart_time - pass_time
                            next_tb = a
                        else:
                            if cur_agv.depart_time - pass_time < min_time_diff_tb:
                                min_time_diff_tb = cur_agv.depart_time - pass_time
                                next_tb = a
                else:
                    #DTA
                    if cur_agv.depart_time <= pass_time and cur_agv.return_time > pass_time:
                        now_dta = a
                    elif cur_agv.depart_time > pass_time:
                        if next_dta < 0:
                            min_time_diff_dta = cur_agv.depart_time - pass_time
                            next_dta = a
                        else:
                            if cur_agv.depart_time - pass_time < min_time_diff_dta:
                                min_time_diff_dta = cur_agv.depart_time - pass_time
                                next_dta = a                      

            tabs = [now_tb, next_tb, now_dta, next_dta]            
            for i in range(0, 4):
                fi = i //2 # fi = 0 TB, fi = 1 DTA
                if i % 2 == 0:
                    # current agv
                    # draw departure and return time
                    if tabs[i] >= 0:
                        cur_agv = day_agvs[tabs[i]]
                        if cur_jobs[fi] != cur_agv.no:
                            cur_jobs[fi] = cur_agv.no
                            if fi == 0:
                                winsound.PlaySound('src\\tb.wav', winsound.SND_FILENAME | winsound.SND_ASYNC)
                            else:
                                winsound.PlaySound('src\\dta.wav', winsound.SND_FILENAME | winsound.SND_ASYNC)
                        pil_draw.text((ox + spacing, oy + row_height*tab_rows[i]), '%d'%(cur_agv.no), (0,0,0), font)
                        pil_draw.text((ox + spacing*2, oy + row_height*tab_rows[i]), time2str(cur_agv.depart_time, day_str_clock)+' - '+time2str(cur_agv.return_time, day_str_clock), (0,0,0), font)
                else:
                    # next agv
                    # draw cargo list
                    if tabs[i] >= 0:
                        next_agv = day_agvs[tabs[i]]
                        pil_draw.text((ox + spacing, oy + row_height*tab_rows[i]), '%d'%(next_agv.no), (0,0,0), font)
                        pil_draw.text((ox + spacing*2, oy + row_height*tab_rows[i]), time2str(next_agv.depart_time, day_str_clock)+' - '+time2str(next_agv.return_time, day_str_clock), (0,0,0), font)
                        max_box_num = 0
                        for bc in next_agv.column_cargos:
                            if len(bc) > max_box_num:
                                max_box_num = len(bc)
                        if max_box_num > 7:
                            print 'high dock'
                            max_box_num = 7
                        dock_col = col*2 / 4
                        for j in range(0, 4):
                            for k in range(0, max_box_num):
                                if k >= len(next_agv.column_cargos[j]):
                                    break
                                if fi == 0:
                                    #TB
                                    pil_draw.text((ox + dock_col*j, oy + row_height*(tab_rows[i]+max_box_num-k)), next_agv.column_cargos[j][k], (0,112,192), font)
                                else:
                                    #DTA
                                    pil_draw.text((ox + dock_col*j, oy + row_height*(tab_rows[i]+max_box_num-k)), next_agv.column_cargos[j][k], (0,143,83), font) 
          
            
            # update work station contents
            for i in range(0, 3):
                if now_stat[i] >= 0:
                    if pass_time > day_ops[now_stat[i]].end_time:
                        now_stat[i] = -1
            next_stat = [-1, -1, -1]
            min_time_diffs = [0, 0, 0] 
            for i in range(0, len(day_ops)):
                tmp_op = day_ops[i]
                stat_in = tmp_op.lane - 1 #index of which lane
                if pass_time >= tmp_op.str_time and pass_time < tmp_op.end_time:
                    now_stat[stat_in] = i
                elif tmp_op.str_time > pass_time:
                    if min_time_diffs[stat_in] == 0:
                        min_time_diffs[stat_in] = tmp_op.str_time - pass_time
                        next_stat[stat_in] = i
                    else:
                        if tmp_op.str_time - pass_time < min_time_diffs[stat_in]:
                            min_time_diffs[stat_in] = tmp_op.str_time - pass_time
                            next_stat[stat_in] = i
            for i in range(0, 3):
                pil_draw.text((ox + col*(i+2), oy + row_height), 'Now', (0,176,240), font)
                if now_stat[i] >= 0:
                    if cur_jobs[i+2] != now_stat[i]:
                        cur_jobs[i+2] = now_stat[i]
                        winsound.PlaySound('src\\lane%d.wav'%(i+1), winsound.SND_FILENAME | winsound.SND_ASYNC)
                    cur_op = day_ops[now_stat[i]]
                    if cur_op.type == 3:
                        #kitting
                        tmp_str = u'%s 配料'%(cur_op.label)
                        pil_draw.text((ox + col*(i+2) + spacing-10, oy + row_height), tmp_str, (255,0,0), font)
                        pil_draw.text((ox + col*(i+2) + spacing*2, oy + row_height), time2str(cur_op.str_time, day_str_clock)+' - '+time2str(cur_op.end_time, day_str_clock), (0,0,0), font)
                        pil_draw.text((ox + col*(i+2), oy + row_height*2), cur_op.work[0], (0,0,0), font)
                        tmp_str = '数量： %d'%len(cur_op.work)
                        tmp_str = tmp_str.decode('utf8')
                        pil_draw.text((ox + col*(i+2), oy + row_height*3), tmp_str, (0,0,0), font)
                    elif cur_op.type == 2:
                        #inbound
                        tmp_str = '入库'
                        tmp_str = tmp_str.decode('utf8')
                        pil_draw.text((ox + col*(i+2) + spacing-10, oy + row_height), tmp_str, (0,176,80), font)
                        pil_draw.text((ox + col*(i+2) + spacing*2, oy + row_height), time2str(cur_op.str_time, day_str_clock)+' - '+time2str(cur_op.end_time, day_str_clock), (0,0,0), font)
                        tmp_str = '数量： %d'%((cur_op.end_time - cur_op.str_time)/50)#unit inbound time 50s
                        tmp_str = tmp_str.decode('utf8')
                        pil_draw.text((ox + col*(i+2), oy + row_height*2), tmp_str, (0,0,0), font)
                    elif cur_op.type == 1:
                        #outbound
                        tmp_str = u'%s 出库'%(cur_op.label)
                        #tmp_str = tmp_str.decode('utf8')
                        pil_draw.text((ox + col*(i+2) + spacing-10, oy + row_height), tmp_str, (0,0,255), font)
                        pil_draw.text((ox + col*(i+2) + spacing*2, oy + row_height), time2str(cur_op.str_time, day_str_clock)+' - '+time2str(cur_op.end_time, day_str_clock), (0,0,0), font)
                        col_index = []
                        for ag in day_agvs:
                            if ag.type == cur_op.agv_type and ag.no == cur_op.agv_no:                                
                                for j in range(0, len(cur_op.work)):
                                    box_found = False
                                    for k in range(0, 4):
                                        if len(ag.column_cargos[k]) > 0:
                                            for ag_box in ag.column_cargos[k]:
                                                if ag_box == cur_op.work[j]:
                                                    box_found = True
                                                    col_index.append(k+1)
                                                    break;
                                        if box_found:
                                            break;
                        cargo_strs = []
                        if len(col_index) == len(cur_op.work):
                            #draw work list with column index                            
                            for k in range(1, 5):
                                for m in range(0, len(col_index)):
                                    if k == col_index[m]:
                                        cargo_strs.append('%d '%k + cur_op.work[m])
                        else:
                            print 'operation & agv work does not match'
                            cargo_strs = cur_op.work
                        for j in range(0, len(cargo_strs)):
                            if j < 18:
                                pil_draw.text((ox + col*(i+2), oy + row_height*(j+3)), cargo_strs[j], (0,0,0), font)
                                
                pil_draw.text((ox + col*(i+2), oy + row_height*21), 'Next', (255,51,204), font)
                if next_stat[i] >= 0:
                    next_op = day_ops[next_stat[i]]
                    if next_op.type == 3:
                        #kitting
                        tmp_str = u'%s 配料'%(next_op.label)
                        tmp_str = '配料'
                        tmp_str = tmp_str.decode('utf8')
                        pil_draw.text((ox + col*(i+2) + spacing-10, oy + row_height*21), tmp_str, (255,0,0), font)
                        pil_draw.text((ox + col*(i+2) + spacing*2, oy + row_height*21), time2str(next_op.str_time, day_str_clock)+' - '+time2str(next_op.end_time, day_str_clock), (0,0,0), font)
                    elif next_op.type == 2:
                        #inbound
                        tmp_str = '入库'
                        tmp_str = tmp_str.decode('utf8')
                        pil_draw.text((ox + col*(i+2) + spacing-10, oy + row_height*21), tmp_str, (0,176,80), font)
                        pil_draw.text((ox + col*(i+2) + spacing*2, oy + row_height*21), time2str(next_op.str_time, day_str_clock)+' - '+time2str(next_op.end_time, day_str_clock), (0,0,0), font)
                    elif next_op.type == 1:
                        #tmp_str = '%s出库'%(next_op.label)
                        tmp_str = u'%s 出库'%(next_op.label)
                        pil_draw.text((ox + col*(i+2) + spacing-10, oy + row_height*21), tmp_str, (0,0,255), font)
                        pil_draw.text((ox + col*(i+2) + spacing*2, oy + row_height*21), time2str(next_op.str_time, day_str_clock)+' - '+time2str(next_op.end_time, day_str_clock), (0,0,0), font)  
                  

            
            img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
            cv2.imshow('Gantt', img)   
#             cv2.imwrite('c:\\tmp\\clip\\frame%.5d.jpg'%count, img)
#             count = count + 1       
    
                
    