function blk = pickTarget(blocks)
[~,i] = max([blocks.area]);  % largest blob
blk = blocks(i);
end
