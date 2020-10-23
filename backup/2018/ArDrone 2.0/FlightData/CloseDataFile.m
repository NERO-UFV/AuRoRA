function CloseDataFile(journal)


% Close the data file created for the respective object.
for idx = 1:size(journal,2)
    fclose(journal{idx}.DataFile);
    disp([journal{idx}.Type,' Data Saved.'])
end
end