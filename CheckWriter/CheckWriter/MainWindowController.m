//
//  MainWindowController.m
//  CheckWriter
//
//  Created by Jesse Riddle on 4/27/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#import "MainWindowController.h"

@implementation MainWindowController
@synthesize selectAccount;
@synthesize deleteSelectedAccount;
@synthesize checkNo;
@synthesize checkDate;
@synthesize checkPayee;
@synthesize checkAmount;
@synthesize checkLongAmount;
@synthesize checkMemo;
@synthesize checkPrint;
@synthesize accountDetails;

- (void)awakeFromNib
{
    [selectAccount removeAllItems];
}

- (id)initWithWindow:(NSWindow *)window
{
    self = [super initWithWindow:window];
    if (self) {
        // Initialization code here.
    }
    
    return self;
}

- (void)windowDidLoad
{
    [super windowDidLoad];
    
    // Implement this method to handle any initialization after your window controller's window has been loaded from its nib file.
}

@end
