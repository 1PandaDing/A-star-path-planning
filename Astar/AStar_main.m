%定义2D栅格地图矩阵
X_SCALE=20;
Y_SCALE=9;
%这个数组存储了地图的坐标和每个坐标中的物体（起点，目标，障碍物等）
MAP_2D=2*(ones(X_SCALE,Y_SCALE));

% 通过鼠标点击定义起始点，目标点和障碍物点
% 我们设定障碍物方格值为-1，目标点为0，起始点为1，可通过点为2
n=0;
x_step = 1;
y_step = 1;
axis([1 X_SCALE+1 1 Y_SCALE+1]);
set(gca,'XTick',0:1:20);
set(gca,'YTick',0:1:9);
set (gcf,'Position',[200,200,1500,600], 'color','#E0FFFF')
grid on;
set(gca, 'GridColor', 'k','LineWidth',1.5,'GridAlpha',0.5,'GridColor','r'); 
hold on;
n=0;%障碍物方格的数量

% 手动选择障碍物，起始点和目标点
pause(1);
text_msg=msgbox('请选择目标点');
ht = findobj(text_msg, 'Type', 'text');
set(ht, 'FontSize', 12, 'Unit', 'normal');
uiwait(text_msg,2);
if ishandle(text_msg) == 1
    delete(text_msg);
end
xlabel('请使用鼠标左键点击目标点','Color','black','FontSize',15);
but=0;
while (but ~= 1) %循环直到鼠标左键被点击，目标点被选中
    [point_x,point_y,but]=ginput(1);
end
point_x=floor(point_x);
point_y=floor(point_y);
xTarget=point_x;     % 目标点的横坐标
yTarget=point_y;     % 目标点的纵坐标

MAP_2D(point_x,point_y)=0      ; %更新MAP设目标点数值为0
plot(point_x+.5,point_y+.5,'p','markersize',8,'LineWidth',4,'color','r');
text(point_x+1,point_y+.5,'目标点','FontSize',15)

pause(2);
text_msg=msgbox('请选择障碍物');
ht = findobj(text_msg, 'Type', 'text');
set(ht, 'FontSize', 12, 'Unit', 'normal');
  xlabel('请使用鼠标左键点击障碍物点，并以右键结束最后一个障碍物点','Color','black','FontSize',15);
uiwait(text_msg,10);
if ishandle(text_msg) == 1
    delete(text_msg);
end
while but == 1
    [point_x,point_y,but] = ginput(1);
    point_x=floor(point_x);  %取整数，保证坐标为整数坐标
    point_y=floor(point_y);
    MAP_2D(point_x,point_y)=-1;%   更新MAP设障碍物点数值为-1
    plot(point_x+.5,point_y+.5,'x','markersize',20,'Color','k','LineWidth',3);
end
 
pause(1);

text_msg=msgbox('鼠标左键点击起始点');
ht = findobj(text_msg, 'Type', 'text');
set(ht, 'FontSize', 12, 'Unit', 'normal');
uiwait(text_msg,5);
if ishandle(text_msg) == 1
    delete(text_msg);
end
xlabel('请使用鼠标左键点击起始点 ','Color','black','FontSize',15);
but=0;
while (but ~= 1) % 等到到鼠标左键被点击
    [point_x,point_y,but]=ginput(1);
    point_x=floor(point_x);
    point_y=floor(point_y);
end
xStart=point_x;  %起始点横坐标
yStart=point_y;  %起始点纵坐标
MAP_2D(point_x,point_y)=1;  %更新MAP设起始点数值为-1
 plot(point_x+.5,point_y+.5,'bo','markersize',15,'Color','b','LineWidth',3);
 text(point_x-.5,point_y-.5,'起始点','FontSize',15)
%选择障碍物点结束

%初始化开启列表
OPEN_LIST=[];

%初始化关闭列表
CLOSED_LIST=[];

%将所有障碍物点放入关闭列表CLOSED LIST

counter=1;  %进行计数
for m=1:X_SCALE
    for n=1:Y_SCALE
        if(MAP_2D(m,n) == -1)
            CLOSED_LIST(counter,1)=m; 
            CLOSED_LIST(counter,2)=n; 
            counter=counter+1;
        end
    end
end
CLOSED_COUNT=size(CLOSED_LIST,1);

%将起始节点设置为第一个节点
xPonit=point_x;
yPoint=point_y;
OPEN_COUNT=1;
Road_cost=0;
goal_distance=distance(xPonit,yPoint,xTarget,yTarget);
%distance函数用于计算距离，采用对角距离，图形中允许朝八个方向移动

OPEN_LIST(OPEN_COUNT,:)=openlist_insert(xPonit,yPoint,xPonit,yPoint,Road_cost,goal_distance,goal_distance);
OPEN_LIST(OPEN_COUNT,1)=0;
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED_LIST(CLOSED_COUNT,1)=xPonit;
CLOSED_LIST(CLOSED_COUNT,2)=yPoint;
NoPath=1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A*算法开始工作
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while((xPonit ~= xTarget || yPoint ~= yTarget) && NoPath == 1)
 expand_map=Expand_function(xPonit,yPoint,Road_cost,xTarget,yTarget,CLOSED_LIST,X_SCALE,Y_SCALE);
 exp_count=size(expand_map,1);
 %更新列表，打开后继节点
 %打开格式列表


 for m=1:exp_count
    flag=0;
    for n=1:OPEN_COUNT
        if(expand_map(m,1) == OPEN_LIST(n,2) && expand_map(m,2) == OPEN_LIST(n,3) )
            OPEN_LIST(n,8)=min(OPEN_LIST(n,8),expand_map(m,5)); 
            if OPEN_LIST(n,8)== expand_map(m,5)
                %更新父节点信息
                OPEN_LIST(n,4)=xPonit;
                OPEN_LIST(n,5)=yPoint;
                OPEN_LIST(n,6)=expand_map(m,3);
                OPEN_LIST(n,7)=expand_map(m,4);
            end;
            flag=1;
        end;

    end;
    if flag == 0
        OPEN_COUNT = OPEN_COUNT+1;
        OPEN_LIST(OPEN_COUNT,:)=openlist_insert(expand_map(m,1),expand_map(m,2),xPonit,yPoint,expand_map(m,3),expand_map(m,4),expand_map(m,5));
     end;     %停止向开启列表中加入新元素
 end;%

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %结束While循环
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %找出f(n)最小的节点 

  index_min_node = minimum_fn(OPEN_LIST,OPEN_COUNT,xTarget,yTarget);
  if (index_min_node ~= -1)    
   %将f(n)最小的点设为xNode和yNode
   xPonit=OPEN_LIST(index_min_node,2);
   yPoint=OPEN_LIST(index_min_node,3);
   Road_cost=OPEN_LIST(index_min_node,6);%更新达到父节点的成本
  %将已遍历过的点移动到 List CLOSED中

  CLOSED_COUNT=CLOSED_COUNT+1;
  CLOSED_LIST(CLOSED_COUNT,1)=xPonit;
  CLOSED_LIST(CLOSED_COUNT,2)=yPoint;
  OPEN_LIST(index_min_node,1)=0;
  else
      %无法到达目标点
      NoPath=0;   %跳出循环
  end;
end; 
%一旦算法运行，就会产生最佳路径，从最后一个节点开始（如果它是目标节点）
% 然后确定其父节点，直到到达起始节点。

m=size(CLOSED_LIST,1);
Optimal_path=[];
point_x=CLOSED_LIST(m,1);
point_y=CLOSED_LIST(m,2);
m=1;
Optimal_path(m,1)=point_x;
Optimal_path(m,2)=point_y;
m=m+1;

if ( (point_x == xTarget) && (point_y == yTarget))
    inode=0;
   %遍历OPEN并确定父节点
   parent_x=OPEN_LIST(return_node_index(OPEN_LIST,point_x,point_y),4);%return_node_index函数返回节点的索引
   parent_y=OPEN_LIST(return_node_index(OPEN_LIST,point_x,point_y),5);
   
   while( parent_x ~= xStart || parent_y ~= yStart)
           Optimal_path(m,1) = parent_x;
           Optimal_path(m,2) = parent_y;
           
           inode=return_node_index(OPEN_LIST,parent_x,parent_y);
           parent_x=OPEN_LIST(inode,4);   %返回节点的索引
           parent_y=OPEN_LIST(inode,5);
           m=m+1;
    end;
 n=size(Optimal_path,1);
 %画出规划路径图

 road=plot(Optimal_path(n,1)+.5,Optimal_path(n,2)+.5,'bo','markersize',15,'Color','b','LineWidth',3);
 n=n-1;
 for m=n:-1:1
  pause(.25);
  set(road,'XData',Optimal_path(m,1)+.5,'YData',Optimal_path(m,2)+.5);
 drawnow ;
 end;
 Optimal_path=[Optimal_path;xStart,yStart]
 plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5,'LineWidth',3,'Color','k');
else
 pause(1);
 text_msg=msgbox('无法达到目标点','warn');
 ht = findobj(text_msg, 'Type', 'text');
 set(ht, 'FontSize', 12, 'Unit', 'normal');
 uiwait(text_msg,5);
end

    

