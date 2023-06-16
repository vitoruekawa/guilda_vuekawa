omega_end = omega.Data(end);
v_dq0_end = v_dq0.Data(end,:);
v_dq_end = v_dq0_end(1:2);

i_dq0_end = i_dq0.Data(end,:);
i_dq_end = i_dq0_end(1:2);

i_dc_st_end = i_dc_st.Data(end);

i_s_dq0_end = i_s_dq0.Data(end,:);
i_s_dq_end = i_s_dq0_end(1:2);

m_dq0_end = m_dq.Data(end,:);
m_dq_end = m_dq0_end(1:2);

v_dc_end = v_dc.Data(end);

d_vdc = i_dc_st_end - (v_dc_end / R_dc) - (1/2) * m_dq_end * transpose(i_s_dq_end);
d_i = (-R_f * eye(2) + omega_end * L_f * [0, -1; 1, 0])* transpose(i_s_dq_end) - transpose(v_dq_end) - (1/2) * transpose(m_dq_end) * v_dc_end;
d_vdq = -C_f * omega_end * [0, -1; 1, 0] * transpose(v_dq_end) + transpose(i_s_dq_end) - transpose(i_dq_end);