function df = mat_to_txt(file)
	load(file);
	df = strcat(file, '.txt');
	dlmwrite(df, truth);
end
