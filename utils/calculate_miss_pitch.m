function miss_pitch = calculate_miss_pitch(x_out)

pitch_f = x_out(end, 8);
miss_pitch = 0 - pitch_f;

end