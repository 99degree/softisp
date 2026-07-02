    pub fn add_warp_grid(&mut self, target_width: u32, target_height: u32) -> &mut Self {
        let warp = WarpGridBlock::new(target_width, target_height);
        self.add_block(Box::new(warp))
    }
