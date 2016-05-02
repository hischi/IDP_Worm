function add_muscle_elements(blk,hB,hF,hFz,N)

src = sprintf('muscle_element_lib/Muscle Element');
src_prt = [get_param(hB,'Name') '/RConn1'];
src_prt_Fz = [get_param(hFz,'Name') '/RConn1'];
for i=1:N
        h = add_block(src,[blk '/Muscle Element' num2str(i)],...
            'position',[i*100 42 i*100+60 78]);
        % Connect to previous block
        dst_prt = [get_param(h,'name') '/LConn1'];
        dst_prt_Fz = [get_param(h,'name') '/LConn2'];
        add_line(blk,src_prt,dst_prt);
        add_line(blk,src_prt_Fz,dst_prt_Fz);
        src_prt = [get_param(h,'name') '/RConn1'];
end
set_param(hF,'Position',[ i*100+100 53  i*100+130 67]);
dst_prt = [get_param(hF,'Name') '/RConn1'];
add_line(blk,src_prt,dst_prt);


